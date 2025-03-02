//! Audio driver
//!
//! Uses SPI2 and DMA Channel 1 and respective interrupts
//!

use core::{fmt::Display, mem::size_of};
use log::{debug, error};
use mips_mcu::fmt::virt_to_phys;
use pic32_hal::{
    clock::refclock::{Refclock, Source},
    dma::{DmaChannel, Ops, XferMode},
    int::InterruptSource,
    pac::{self, DMAC1, SPI2},
    spi::{AudioFrameFormat, Proto, Spi},
};

const CLOCK_RATIO: u32 = 256; // ratio Master MCLK to LRCLK

pub struct AudioOut<'a, FRAME> {
    /// ring buffer having P periods of length SIZE
    dma_buffer: &'a mut [FRAME],

    /// Write index. Corresponds to the index of the next frame to be written
    wr_ndx: usize,

    overrun_ctr: usize,

    // Read index. Corresponds to the first frame of the period currently transferred via DMA
    // rd_ndx: AtomicUsize,
    channel: DmaChannel<DMAC1>,
    /// sampling frequency (L/R clock)
    fsamp: u32,

    /// adjustment of reference clock divisor
    clock_adj: i8,

    refclock: Refclock,
}

impl<'a, FRAME: Copy + Display> AudioOut<'a, FRAME> {
    pub fn new(spi: SPI2, dma: DMAC1, refclock: Refclock, dma_buffer: &'a mut [FRAME]) -> Self {
        let fsamp = 48000;
        let clock_adj = 0;
        // initialize reference clock
        refclock.select_source(Source::Usbpll).unwrap();
        Self::set_clock(&refclock, fsamp, clock_adj);
        refclock.output_enable();
        refclock.enable();

        let mut channel = DmaChannel::channel1(dma);
        channel.set_start_event(Some(InterruptSource::SPI2_TX));
        let buf_ptr = dma_buffer.as_mut_ptr();
        channel.set_source(virt_to_phys(buf_ptr), size_of_val(dma_buffer));

        let spibuf = unsafe { &((*pac::SPI2::ptr()).buf) as *const _ as *mut u32 };
        let dmaaddr = virt_to_phys(spibuf);
        channel.set_dest(dmaaddr, 2);
        channel.set_cell_size(2);
        unsafe {
            channel.enable(XferMode::Auto);
        }

        let _spi = Spi::spi2(
            spi,
            Proto::AudioI2s(AudioFrameFormat::F64S16),
            CLOCK_RATIO / 64,
        );

        Self {
            dma_buffer,
            wr_ndx: 0,
            overrun_ctr: 0,
            channel,
            fsamp,
            clock_adj,
            refclock,
        }
    }

    /// Set the clock frequency based on the sampling frequency (L/R clock) in
    /// Hertz and an adjustment value for dynamic buffer management
    fn set_clock(refclock: &Refclock, fsamp: u32, adj: i8) {
        let master_clock = fsamp * CLOCK_RATIO; // 12.288 MHz
        let div = 256 * 96_000_000 / master_clock as i64;
        let div_adj = div as i32 + adj as i32;
        debug!("set_clock(fsamp = {fsamp}, adj = {adj}, div = {div_adj:06x})");
        critical_section::with(|_cs| refclock.set_divisor(div_adj as u32)).unwrap_or_else(|e| {
            error!("could not set divisor to {div_adj}: {e:?}");
        });
    }

    pub fn clock_tuning(&mut self) {
        let fill_ratio = self.buf_len() as f32 / self.buf_capacity() as f32;
        let new_clock_adj = if fill_ratio > 0.6 {
            -1
        } else if fill_ratio < 0.4 {
            1
        } else {
            self.clock_adj
        };
        if new_clock_adj != self.clock_adj {
            self.clock_adj = new_clock_adj;
            Self::set_clock(&self.refclock, self.fsamp, self.clock_adj);
        }
    }

    /// Write audio data to ring buffer.
    /// The average data rate must correspond to the sampling rate
    pub fn write(&mut self, frames: &[FRAME]) {
        let free = self.dma_buffer.len() - self.buf_len();
        let n_frames = if frames.len() <= free {
            frames.len()
        } else {
            self.overrun_ctr += 1;
            free
        };
        for frame in &frames[..n_frames] {
            self.dma_buffer[self.wr_ndx] = *frame;
            self.wr_ndx += 1;
            if self.wr_ndx >= self.dma_buffer.len() {
                self.wr_ndx = 0;
            }
        }
    }

    /// current dma buffer length in frames
    pub fn buf_len(&self) -> usize {
        let rd_ndx = self.channel.source_pointer().address() / size_of::<FRAME>();
        if self.wr_ndx >= rd_ndx {
            self.wr_ndx - rd_ndx
        } else {
            self.dma_buffer.len() - rd_ndx + self.wr_ndx
        }
    }

    /// capacity of the dma buffer in frames
    pub fn buf_capacity(&self) -> usize {
        self.dma_buffer.len()
    }

    /// overrun counter
    pub fn overrun_ctr(&self) -> usize {
        self.overrun_ctr
    }
}
