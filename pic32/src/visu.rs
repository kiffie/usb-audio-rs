//! Audio visualization

use pic32_hal::dma::Ops as DmaOps;
use pic32_hal::i2c::Ops as I2cOps;

use embedded_hal_02::blocking::i2c::Write as I2cWrite;

use mips_mcu::fmt::virt_to_phys;

use ssd1306::{command, Builder, I2CDIBuilder};

use num_complex::Complex;

use fixed_fft::fft_radix2_real_q15;
use log::error;

const FBLEN: usize = 128 * 64 / 8;
const FFT_LDN: usize = 9;
const FFT_N: usize = 1 << FFT_LDN;

pub const BLOCK_LENGTH: usize = FFT_N;

const VISU_ALPHA: u32 = 65536 / 8 * 4;
const VISU_DECAY: u32 = 65536 / 8;

const VISU_SPEC_BINS: usize = 16;
const VISU_WIDTH: usize = 128;
const VISU_HEIGHT: usize = 64;
const VISU_PAGESIZE: usize = 8;
const VISU_BAR_WIDTH: usize = VISU_WIDTH / VISU_SPEC_BINS / 4 * 3;

// Hamming window, size: 512 points
static HAMMING: [u16; FFT_N] = [
    5242, 5245, 5251, 5263, 5279, 5299, 5324, 5354, 5388, 5427, 5470, 5518, 5570, 5627, 5688, 5754,
    5824, 5899, 5978, 6061, 6149, 6242, 6339, 6440, 6546, 6656, 6770, 6888, 7011, 7139, 7270, 7406,
    7546, 7690, 7839, 7991, 8148, 8309, 8474, 8643, 8816, 8993, 9174, 9359, 9548, 9741, 9937,
    10138, 10342, 10550, 10762, 10978, 11197, 11420, 11647, 11877, 12111, 12348, 12589, 12833,
    13081, 13332, 13586, 13844, 14105, 14369, 14636, 14907, 15180, 15457, 15736, 16019, 16304,
    16593, 16884, 17178, 17475, 17774, 18077, 18381, 18689, 18998, 19311, 19626, 19943, 20262,
    20584, 20908, 21234, 21563, 21893, 22225, 22560, 22896, 23235, 23575, 23917, 24260, 24606,
    24953, 25301, 25651, 26003, 26356, 26710, 27066, 27422, 27781, 28140, 28500, 28862, 29224,
    29587, 29951, 30316, 30682, 31049, 31416, 31783, 32152, 32521, 32890, 33259, 33629, 33999,
    34370, 34740, 35111, 35482, 35852, 36223, 36593, 36964, 37334, 37703, 38073, 38442, 38810,
    39178, 39546, 39913, 40279, 40644, 41009, 41372, 41735, 42097, 42458, 42818, 43177, 43534,
    43890, 44245, 44599, 44951, 45302, 45651, 45999, 46345, 46690, 47032, 47373, 47713, 48050,
    48385, 48719, 49050, 49380, 49707, 50032, 50355, 50676, 50994, 51310, 51624, 51935, 52243,
    52549, 52853, 53154, 53452, 53747, 54040, 54330, 54617, 54901, 55182, 55460, 55735, 56007,
    56276, 56541, 56804, 57063, 57319, 57572, 57821, 58067, 58310, 58549, 58784, 59016, 59245,
    59469, 59691, 59908, 60122, 60332, 60538, 60741, 60939, 61134, 61325, 61512, 61695, 61874,
    62049, 62220, 62387, 62550, 62709, 62863, 63014, 63160, 63302, 63440, 63574, 63703, 63828,
    63949, 64066, 64178, 64286, 64389, 64488, 64583, 64673, 64759, 64840, 64917, 64990, 65058,
    65121, 65180, 65235, 65285, 65330, 65371, 65407, 65439, 65467, 65489, 65508, 65521, 65530,
    65535, 65535, 65530, 65521, 65508, 65489, 65467, 65439, 65407, 65371, 65330, 65285, 65235,
    65180, 65121, 65058, 64990, 64917, 64840, 64759, 64673, 64583, 64488, 64389, 64286, 64178,
    64066, 63949, 63828, 63703, 63574, 63440, 63302, 63160, 63014, 62863, 62709, 62550, 62387,
    62220, 62049, 61874, 61695, 61512, 61325, 61134, 60939, 60741, 60538, 60332, 60122, 59908,
    59691, 59469, 59245, 59016, 58784, 58549, 58310, 58067, 57821, 57572, 57319, 57063, 56804,
    56541, 56276, 56007, 55735, 55460, 55182, 54901, 54617, 54330, 54040, 53747, 53452, 53154,
    52853, 52549, 52243, 51935, 51624, 51310, 50994, 50676, 50355, 50032, 49707, 49380, 49050,
    48719, 48385, 48050, 47713, 47373, 47032, 46690, 46345, 45999, 45651, 45302, 44951, 44599,
    44245, 43890, 43534, 43177, 42818, 42458, 42097, 41735, 41372, 41009, 40644, 40279, 39913,
    39546, 39178, 38810, 38442, 38073, 37703, 37334, 36964, 36593, 36223, 35852, 35482, 35111,
    34740, 34370, 33999, 33629, 33259, 32890, 32521, 32152, 31783, 31416, 31049, 30682, 30316,
    29951, 29587, 29224, 28862, 28500, 28140, 27781, 27422, 27066, 26710, 26356, 26003, 25651,
    25301, 24953, 24606, 24260, 23917, 23575, 23235, 22896, 22560, 22225, 21893, 21563, 21234,
    20908, 20584, 20262, 19943, 19626, 19311, 18998, 18689, 18381, 18077, 17774, 17475, 17178,
    16884, 16593, 16304, 16019, 15736, 15457, 15180, 14907, 14636, 14369, 14105, 13844, 13586,
    13332, 13081, 12833, 12589, 12348, 12111, 11877, 11647, 11420, 11197, 10978, 10762, 10550,
    10342, 10138, 9937, 9741, 9548, 9359, 9174, 8993, 8816, 8643, 8474, 8309, 8148, 7991, 7839,
    7690, 7546, 7406, 7270, 7139, 7011, 6888, 6770, 6656, 6546, 6440, 6339, 6242, 6149, 6061, 5978,
    5899, 5824, 5754, 5688, 5627, 5570, 5518, 5470, 5427, 5388, 5354, 5324, 5299, 5279, 5263, 5251,
    5245, 5242,
];

static FFT_NDX_MAP: [usize; VISU_SPEC_BINS] =
    [0, 1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 64, 96, 128, 191, 255];

#[derive(Debug, Clone)]
pub enum Error {
    InvalidDataLength,
    WouldBlock,
}

pub struct Visualization<I: I2cOps, D: DmaOps> {
    i2c: I,
    addr: u8,
    dma: D,

    samples_fft_in: [i16; FFT_N],
    samples_avail: bool,
    framebuf: [u8; FBLEN],

    spec_avg: [u16; VISU_SPEC_BINS],
    spec_peak: [u16; VISU_SPEC_BINS],
}

impl<I: I2cOps + I2cWrite, D: DmaOps> Visualization<I, D> {
    pub fn new(i2c: I, dma: D) -> Visualization<I, D> {
        let addr = 0x3c;
        let interface = I2CDIBuilder::new().with_i2c_addr(addr).init(i2c);

        //let interface = Interface::new(i2c, dma, addr);
        let mut disp = Builder::new().connect(interface);
        disp.init_with_mode(command::AddrMode::Horizontal).unwrap();
        let i2c = disp.release().release();
        Visualization {
            i2c,
            addr,
            dma,
            framebuf: [0; FBLEN],
            samples_fft_in: [0; FFT_N],
            samples_avail: false,
            spec_avg: [0; VISU_SPEC_BINS],
            spec_peak: [u16::MAX; VISU_SPEC_BINS],
        }
    }

    /// Input a block of audio samples.
    ///
    /// The length of the block must be BLOCK_LENGTH
    pub fn input(&mut self, samples: &[i16]) -> Result<(), Error> {
        if samples.len() != BLOCK_LENGTH {
            return Err(Error::InvalidDataLength);
        }
        if self.samples_avail {
            return Err(Error::WouldBlock);
        }
        self.samples_fft_in.copy_from_slice(samples);
        self.samples_avail = true;
        Ok(())
    }

    pub fn tasks(&mut self) {
        let mut spectrum = [Complex::new(0, 0); FFT_N / 2 + 1];
        if self.samples_avail {
            for (s, h) in self.samples_fft_in.iter_mut().zip(HAMMING.iter()) {
                let mut w = *s as i32 * *h as i32;
                w /= 65536;
                *s = w as i16;
            }
            fft_radix2_real_q15(&mut self.samples_fft_in, &mut spectrum, true).unwrap();
            #[allow(clippy::needless_range_loop)]
            for j in 0..VISU_SPEC_BINS {
                let mut val;
                if j > 0 {
                    val = 0;
                    for s in &spectrum[FFT_NDX_MAP[j - 1]..FFT_NDX_MAP[j]] {
                        val += s.re.unsigned_abs() as u32;
                    }
                    val /= (FFT_NDX_MAP[j] - FFT_NDX_MAP[j - 1]) as u32;
                } else {
                    val = 2 * spectrum[FFT_NDX_MAP[j]].re.unsigned_abs() as u32
                }
                let avg =
                    (VISU_ALPHA * val + (65536 - VISU_ALPHA) * self.spec_avg[j] as u32) / 65536;
                self.spec_avg[j] = avg as u16;
                self.spec_peak[j] = self.spec_peak[j].max(val as u16);
            }
            self.samples_avail = false;
        }
    }

    /// Start transmission via DMA
    fn fb_start_transmit(&mut self) {
        // complete previous transfer
        self.i2c.stop();

        self.i2c
            .transmit(&[
                self.addr << 1,
                0x00,
                0x21,
                0,
                (VISU_WIDTH - 1) as u8,
                0x22,
                0,
                ((VISU_HEIGHT >> 3) - 1) as u8,
            ])
            .unwrap();
        self.i2c.stop();

        // transmit I2C address and data/control byte via PIO
        self.i2c.transmit(&[self.addr << 1, 0x40]).unwrap();
        // initiate DMA transfer of frame buffer
        unsafe {
            self.i2c
                .transmit_dma(
                    &mut self.dma,
                    virt_to_phys(&mut self.framebuf),
                    self.framebuf.len(),
                )
                .unwrap();
        }
    }

    fn fb_setbyte(&mut self, col: usize, page: usize, data: u8) {
        let ndx = 128 * page + col;
        if ndx >= FBLEN {
            error!("invalid display address: col = {}, page = {}", col, page);
            return;
        }
        self.framebuf[ndx] = data;
    }

    /// Start display update via DMA
    pub fn start_update(&mut self) {
        for byte in self.framebuf.iter_mut() {
            *byte = 0;
        }
        self.i2c.stop();
        //ssd1306_lcd.set_block(0, 0, 0);

        for i in 0..VISU_SPEC_BINS {
            let mut level = if self.spec_avg[i] > 0 {
                32 - (self.spec_avg[i] as u32).leading_zeros()
            } else {
                0
            };
            level *= VISU_HEIGHT as u32 / 16;

            let mut peak = if self.spec_peak[i] > 0 {
                32 - (self.spec_peak[i] as u32).leading_zeros()
            } else {
                0
            };
            peak *= VISU_HEIGHT as u32 / 16;

            let col = i * (VISU_WIDTH / VISU_SPEC_BINS);
            let level_pages = level / VISU_PAGESIZE as u32;
            let level_remainder = level % VISU_PAGESIZE as u32;

            let peak_pages = peak / VISU_PAGESIZE as u32;
            let peak_remainder = peak % VISU_PAGESIZE as u32;

            for j in 0..(VISU_HEIGHT / VISU_PAGESIZE) {
                let mut pattern = match level_pages {
                    lp if lp > j as u32 => 0xff,
                    lp if lp == j as u32 => (0xffu32 << (8 - level_remainder)) as u8,
                    _ => 0,
                };
                if peak_pages == j as u32 && j != 0 {
                    pattern |= (0x80u32 >> peak_remainder) as u8;
                }
                if pattern != 0 {
                    for k in 0..VISU_BAR_WIDTH {
                        self.fb_setbyte(col + k, VISU_HEIGHT / VISU_PAGESIZE - 1 - j, pattern);
                    }
                }
            }
            self.spec_peak[i] = (self.spec_peak[i] as u32 * (65536 - VISU_DECAY) / 65536) as u16;
        }
        self.fb_start_transmit();
    }

    /// Checks if Display update is complete
    pub fn update_complete(&self) -> bool {
        !self.dma.is_enabled()
    }
}

// struct Interface<O: I2cOps, D: DmaOps> {
//     i2c: O,
//     dma: D,
//     addr: u8,
// }

// impl<O: I2cOps, D: DmaOps> Interface<O, D> {
//     pub fn new(i2c: O, dma: D, addr: u8) -> Self {
//         Interface { i2c, dma, addr }
//     }

//     pub fn release(self) -> (O, D) {
//         (self.i2c, self.dma)
//     }
// }

// impl<O: I2cOps, D: DmaOps> WriteOnlyDataCommand for Interface<O, D> {

//     fn send_commands(&mut self, cmd: DataFormat<'_>) -> Result<(), DisplayError> {
//         match cmd {
//             DataFormat::U8(slice) => {
//                 debug!("slice (cmd): {:?}", slice);
//                 self.i2c.transmit(&[self.addr << 1]).unwrap();
//                 let cmd_byte = [0x00u8; 1];
//                 self.i2c.transmit(&cmd_byte).unwrap();
//                 self.i2c.transmit(slice).unwrap();
//                 self.i2c.stop();
//                 Ok(())
//             }
//             _ => Err(DisplayError::DataFormatNotImplemented),
//         }
//     }

//     fn send_data(&mut self, buf: DataFormat<'_>) -> Result<(), DisplayError> {
//         match buf {
//             DataFormat::U8(slice) => {
//                 debug!("slice: {:?}", slice);
//                 self.i2c.transmit(&[self.addr << 1]).unwrap();
//                 let cmd_byte = [0x40u8; 1];
//                 self.i2c.transmit(&cmd_byte).unwrap();
//                 self.i2c.transmit(slice).unwrap();
//                 self.i2c.stop();
//                 Ok(())
//             },
//             _ => {
//                 debug!("send_data: format not implemented");
//                 Err(DisplayError::DataFormatNotImplemented)
//             },
//         }
//     }
// }
