//! USB Audio

#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::{boxed::Box, collections::VecDeque, vec::Vec};
use cfg_if::cfg_if;
use core::{
    cell::{Cell, RefCell},
    sync::atomic::{AtomicBool, Ordering},
};
use critical_section::Mutex;
use embedded_hal::{delay::DelayNs, digital::OutputPin};
use kiffieboot::{DfuRuntimeClass, Kiffieboot};
use log::{debug, info, LevelFilter};
use mips_mcu::interrupt as irq;
use mips_mcu_alloc::MipsMcuHeap;
use mips_rt::entry;
use pic32_hal::{
    clock::Osc,
    coretimer,
    coretimer::Delay,
    dma,
    gpio::GpioExt,
    i2c::{Fscl, I2c},
    int, pac,
    pps::{MapPin, PpsExt},
    time::U32Ext,
    usb::UsbBus,
};
use usb_device::prelude::*;
use usb_log::log_buffer::LogBuffer;
use usb_log::usb_log_channel::UsbLogChannel;

#[cfg(feature = "pic32mx2x4fxxxb")]
use pic32_config_sector::pic32mx2x4::*;
#[cfg(feature = "pic32mx2xxfxxxb")]
use pic32_config_sector::pic32mx2xx::*;
use usbd_audio::{AudioClassBuilder, Format, StreamConfig, TerminalType};

#[cfg(not(feature = "panic-usb"))]
use {
    core::fmt::Write,
    pic32_hal::{
        pps::NoPin,
        uart::{Uart, CONFIG_115200_8N1},
    },
};

#[cfg(feature = "ssd1306")]
mod visu;

mod audio;
use audio::AudioOut;

mod usb_isr;
use usb_isr::UsbIsr;
use visu::BLOCK_LENGTH;

// Boot loader
#[cfg(feature = "pic32mx2xxfxxxb")]
#[link_section = ".bootloader"]
#[used]
pub static BOOT_LOADER: [u8; 3056] = *include_bytes!("../boot/kb3k-dfu-mx2xx-48mhz.bin");

#[cfg(feature = "pic32mx2x4fxxxb")]
#[link_section = ".bootloader"]
#[used]
pub static BOOT_LOADER: [u8; 3056] = *include_bytes!("../boot/kb3k-dfu-mx2x4-72mhz.bin");

// PIC32 configuration registers for PIC32MX270 (48 MHz)
#[cfg(feature = "pic32mx2xxfxxxb")]
#[link_section = ".configsfrs"]
#[used]
pub static CONFIGSFRS: ConfigSector = ConfigSector::default()
    // DEVCFG3
    .FVBUSONIO(FVBUSONIO::OFF)
    .FUSBIDIO(FUSBIDIO::OFF)
    // DEVCFG2
    .FPLLODIV(FPLLODIV::DIV_2)
    .UPLLEN(UPLLEN::ON)
    .UPLLIDIV(UPLLIDIV::DIV_2)
    .FPLLMUL(FPLLMUL::MUL_24)
    .FPLLIDIV(FPLLIDIV::DIV_2)
    // DEVCFG 1
    .FWDTEN(FWDTEN::OFF)
    .WDTPS(WDTPS::PS1048576)
    .FPBDIV(FPBDIV::DIV_1)
    .POSCMOD(POSCMOD::XT)
    .FSOSCEN(FSOSCEN::OFF)
    .FNOSC(FNOSC::PRIPLL)
    // DEVCFG 0
    .JTAGEN(JTAGEN::OFF)
    .build();

// PIC32 configuration registers for PIC32MX274 (72 MHz)
#[cfg(feature = "pic32mx2x4fxxxb")]
#[link_section = ".configsfrs"]
#[used]
pub static CONFIGSFRS: ConfigSector = ConfigSector::default()
    // DEVCFG3
    .FUSBIDIO(FUSBIDIO::OFF)
    // DEVCFG2
    .FDSEN(FDSEN::OFF)
    .FPLLODIV(FPLLODIV::DIV_2)
    .UPLLEN(UPLLEN::ON)
    .UPLLIDIV(UPLLIDIV::DIV_2)
    .FPLLICLK(FPLLICLK::PLL_POSC)
    .FPLLMUL(FPLLMUL::MUL_18)
    .FPLLIDIV(FPLLIDIV::DIV_1)
    // DEVCFG1
    .FWDTEN(FWDTEN::OFF)
    .WDTPS(WDTPS::PS1048576)
    .FPBDIV(FPBDIV::DIV_1)
    .POSCMOD(POSCMOD::XT)
    .FSOSCEN(FSOSCEN::OFF)
    .FNOSC(FNOSC::SPLL)
    // DEVCFG0
    .JTAGEN(JTAGEN::OFF)
    .build();

#[global_allocator]
static ALLOCATOR: MipsMcuHeap = MipsMcuHeap::empty();

static LOG_BUFFER: LogBuffer<1024> = LogBuffer::new();

#[cfg(not(feature = "panic-usb"))]
static PANIC_TX: Mutex<RefCell<Option<pic32_hal::uart::Tx<pac::UART2>>>> =
    Mutex::new(RefCell::new(None));

type AudioFrame = i16;
const AUDIO_BUFSIZE: usize = 2 * 48000 / 10; // buffer for 100ms
static AUDIO_OUT: Mutex<RefCell<Option<AudioOut<AudioFrame>>>> = Mutex::new(RefCell::new(None));
static USB_AUDIO_ACTIVE: AtomicBool = AtomicBool::new(false);

/// This queue contains Vec<AudioFrame> elements of length visu::BLOCK_LENGTH
static VISU_BUFFER: Mutex<RefCell<VecDeque<Vec<AudioFrame>>>> =
    Mutex::new(RefCell::new(VecDeque::new()));

fn usb_init(usb: pac::USB, int: &int::Int, _timer: &mut Delay) -> UsbIsr {
    let usb_bus = Box::leak(Box::new(UsbBus::new(usb)));
    let mut usb_audio = Box::new(
        AudioClassBuilder::new()
            .output(
                StreamConfig::new_discrete(Format::S16le, 2, &[48000], TerminalType::OutHeadphones)
                    .unwrap(),
            )
            .build(usb_bus)
            .unwrap(),
    );
    let mut log_channel = Box::new(UsbLogChannel::new(usb_bus, &LOG_BUFFER));
    let mut dfu_rt = Box::new(DfuRuntimeClass::new(usb_bus, Kiffieboot::default()));
    let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::new(LangID::EN)
            .manufacturer("Kiffie Labs")
            .product("Audio port")])
        .unwrap()
        .build();

    let mut ctr = 0;
    #[cfg(feature = "ssd1306")]
    let mut visu_buf = Cell::new(Vec::with_capacity(BLOCK_LENGTH));
    UsbIsr::new(int, move || {
        usb_dev.poll(&mut [usb_audio.as_mut(), log_channel.as_mut(), dfu_rt.as_mut()]);
        USB_AUDIO_ACTIVE.store(
            usb_audio.output_alt_setting().unwrap_or_default() > 0,
            Ordering::Relaxed,
        );
        let mut buf = [0u8; 1024];
        if let Ok(len) = usb_audio.read(&mut buf) {
            let mut iter = buf.iter().take(len);
            'bufcpy: loop {
                let mut left: AudioFrame = 0;
                for shift in [0, 8] {
                    if let Some(byte) = iter.next() {
                        left |= (*byte as AudioFrame) << shift;
                    } else {
                        break 'bufcpy;
                    }
                }
                let mut right: AudioFrame = 0;
                for shift in [0, 8] {
                    if let Some(byte) = iter.next() {
                        right |= (*byte as AudioFrame) << shift;
                    } else {
                        break 'bufcpy;
                    }
                }
                critical_section::with(|cs| {
                    if let Some(ao) = AUDIO_OUT.borrow_ref_mut(cs).as_mut() {
                        ao.write(&[left, right]);
                    }
                    #[cfg(feature = "ssd1306")]
                    {
                        visu_buf.get_mut().push(left);
                        if visu_buf.get_mut().len() >= BLOCK_LENGTH {
                            let full = visu_buf.replace(Vec::with_capacity(BLOCK_LENGTH));
                            VISU_BUFFER.borrow_ref_mut(cs).push_back(full);
                        }
                    }
                });
            }

            ctr += 1;
            if ctr >= 1000 {
                ctr = 0;
                let (buf_len, capa, octr) = critical_section::with(|cs| {
                    AUDIO_OUT
                        .borrow_ref(cs)
                        .as_ref()
                        .map(|ao| (ao.buf_len(), ao.buf_capacity(), ao.overrun_ctr()))
                        .unwrap_or_default()
                });
                debug!("len = {len}, buf_len {buf_len}/{capa}, overruns = {octr}");
            }
        }
    })
}

#[entry]
fn main() -> ! {
    static mut AUDIO_BUFFER: [AudioFrame; AUDIO_BUFSIZE] = [0; AUDIO_BUFSIZE];

    // Initialize the allocator BEFORE you use it
    ALLOCATOR.init();

    //configure IO ports for UART1
    let p = pac::Peripherals::take().unwrap();
    let portb = p.PORTB.split();
    let vpins = p.PPS.split();

    // pins for audio interface
    portb
        .rb2
        .into_push_pull_output()
        .map_pin(vpins.outputs.refclko);
    portb
        .rb13
        .into_push_pull_output()
        .map_pin(vpins.outputs.sdo2);
    portb
        .rb14
        .into_push_pull_output()
        .map_pin(vpins.outputs.ss2);
    portb.rb15.into_push_pull_output();

    let mut led = portb.rb5.into_push_pull_output();
    led.set_low().unwrap();

    // setup clock control object
    #[cfg(feature = "pic32mx2xxfxxxb")]
    let (clock, refclock) = Osc::new_with_refclock(p.OSC, 48_000_000_u32.hz());
    #[cfg(feature = "pic32mx2x4fxxxb")]
    let (clock, refclock) = Osc::new_with_refclock(p.CRU, 72_000_000_u32.hz());
    let mut delay = Delay::new(clock.sysclock());

    // initialze UART when needed for panic output
    #[cfg(not(feature = "panic-usb"))]
    {
        let rxd = NoPin::new().map_pin(vpins.inputs.u2rx);
        let txd = portb
            .rb0
            .into_push_pull_output()
            .map_pin(vpins.outputs.u2tx);
        let uart = Uart::uart2(p.UART2, &clock, CONFIG_115200_8N1, rxd, txd);
        let (mut tx, _) = uart.split();
        writeln!(tx, "This UART is used for panic logging.").unwrap();
        critical_section::with(|cs| {
            PANIC_TX.borrow_ref_mut(cs).replace(tx);
        });
    }

    // wait for display to power up
    delay.delay_ms(100);

    log::set_logger(&LOG_BUFFER).unwrap();
    log::set_max_level(LevelFilter::Debug);
    info!(
        "USB Audio output v{} ({})",
        env!("CARGO_PKG_VERSION"),
        env!("BUILD_DATETIME")
    );

    debug!("sysclock = {} Hz", clock.sysclock().0);
    // disable SRAM wait state
    p.BMX.bmxconclr.write(|w| w.bmxwsdrm().bit(true));

    // configure prefetch cache for MX2x4
    #[cfg(feature = "pic32mx2x4fxxxb")]
    {
        let sysclock = clock.sysclock().0;
        let wait_states = ((sysclock - 1) / 18000000) as u8;
        debug!("configuring Flash prefetch cache with {wait_states} wait states");
        p.PCACHE
            .checon
            .write(|w| unsafe { w.prefen().bits(0b11).pfmws().bits(wait_states) });
    }
    unsafe {
        irq::enable();
    }

    // let audio_buffer = unsafe { &mut AUDIO_BUFFER };
    let audio_out = AudioOut::new(p.SPI2, p.DMAC1, refclock, AUDIO_BUFFER);
    critical_section::with(|cs| {
        AUDIO_OUT.borrow_ref_mut(cs).replace(audio_out);
    });

    let mut i2c = Some(I2c::i2c1(p.I2C1, clock.pb_clock(), Fscl::F400KHZ));
    let int = int::Int::new(p.INT);
    let _usb_isr = usb_init(p.USB, &int, &mut delay);

    cfg_if! {
        if #[cfg(feature = "ssd1306")] {
            let mut visu = visu::Visualization::new(i2c.take().unwrap(), dma::DmaChannel::channel2(p.DMAC2));
            let timer = coretimer::Timer::take();
            let ticks_per_ms = clock.sysclock().0 / 2 / 1000;
            let mut visu_update_timer = timer.read_count();
        }
    }

    debug!("starting loop");
    loop {
        cfg_if! {
            if #[cfg(feature = "ssd1306")] {
                visu.tasks();
                if timer.read_count().wrapping_sub(visu_update_timer) >= 50 * ticks_per_ms {
                    if visu.update_complete() {
                        visu.start_update();
                    }
                    visu_update_timer = timer.read_count();
                }
            }
        }
        if USB_AUDIO_ACTIVE.load(Ordering::Relaxed) {
            led.set_high().unwrap();
            let visu_buf = critical_section::with(|cs| {
                AUDIO_OUT
                    .borrow_ref_mut(cs)
                    .as_mut()
                    .unwrap()
                    .clock_tuning();
                VISU_BUFFER.borrow_ref_mut(cs).pop_front()
            });
            if let Some(samples) = visu_buf {
                if visu.input(&samples).is_err() {
                    debug!("VISU buffer overflow");
                }
            }
        } else {
            led.set_low().unwrap();
        }
    }
}

#[alloc_error_handler]
fn alloc_error(layout: core::alloc::Layout) -> ! {
    panic!("Cannot allocate heap memory: {:?}", layout);
}

#[cfg(not(feature = "panic-usb"))]
#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo<'_>) -> ! {
    let mut tx = critical_section::with(|cs| PANIC_TX.take(cs)).unwrap();
    loop {
        write!(tx, "PANIC").ok();
        if let Some(l) = panic_info.location() {
            writeln!(tx, " at {}:{}", l.file(), l.line()).ok();
        }
        writeln!(tx, "{}", panic_info.message()).ok();
        writeln!(tx).ok();
    }
}
