//! USB Interrupt Service Routine
//! Works with an pic32_hal::usb::UsbBus structure

use alloc::boxed::Box;
use core::cell::RefCell;
use core::marker::PhantomData;
use critical_section::Mutex;
use mips_rt::interrupt;
use pic32_hal::int::{Int, IPL4, ISL2};
use pic32_hal::pac::{
    self,
    interrupt::{self, Interrupt, InterruptSource},
};

type Handler = Mutex<RefCell<Option<Box<dyn FnMut() + Send + 'static>>>>;

static HANDLER: Handler = Mutex::new(RefCell::new(None));

/// Handle of USB Interrupt Service Routine
///
/// This is a singleton because... TODO
pub struct UsbIsr {
    _dummy: PhantomData<()>,
}

impl UsbIsr {
    /// Create an IRQ handler.
    ///
    /// Enables the Interrupt and calls the given closure when Interrupts occur.
    /// Panics if an IRQ handler already exists.
    pub fn new<F: FnMut() + Send + 'static>(int: &Int, isr: F) -> Self {
        int.set_ipl(Interrupt::USB_1, IPL4);
        int.set_isl(Interrupt::USB_1, ISL2);
        if critical_section::with(|cs| HANDLER.borrow_ref(cs).is_some()) {
            panic!("IRQ handler already exists");
        }
        critical_section::with(|cs| {
            HANDLER.borrow(cs).replace(Some(Box::new(isr)));
        });
        int.ei(InterruptSource::USB);
        int.set_if(InterruptSource::USB);
        Self {
            _dummy: PhantomData,
        }
    }
}

impl Drop for UsbIsr {
    fn drop(&mut self) {
        let p: pac::Peripherals = unsafe { pac::Peripherals::steal() };
        let int = Int::new(p.INT);
        int.di(InterruptSource::USB);
    }
}

#[interrupt]
fn USB_1() {
    // Accessing the handler directly is safe because we know that there will be
    // no recursive ISR calls and that the main program is using the Mutex
    // safely.
    let ptr = &HANDLER as *const Handler as *mut Handler;
    // let r = ptr.cast_mut();
    let mut handler_cell = unsafe { (*ptr).get_mut() }.borrow_mut();
    let mut handler_option = handler_cell.as_deref_mut();

    if let Some(ref mut handler) = handler_option {
        handler();
    }
    let p: pac::Peripherals = unsafe { pac::Peripherals::steal() };
    let int = Int::new(p.INT);
    int.clear_if(InterruptSource::USB);
}
