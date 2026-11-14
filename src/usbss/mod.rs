//! USB 3.0 SuperSpeed controller (USBSS), RM §27.
//!
//! [`Driver`] is an `embassy-usb` device driver for SuperSpeed (5 Gbit/s).
//! Build the `embassy-usb` configuration with `UsbDeviceSpeed::Super` and a
//! `bcd_usb` of 3.0 or later, and use 1024-byte bulk endpoints, each bursting
//! the `max_burst` of its `EndpointOptions`. The controller has no USB 2.0
//! fallback.
//!
//! The controller mishandles a link error that finds more than a few header
//! packets pending: an OUT endpoint can stall, a control transfer can fail, or
//! one IN packet can carry wrong data. That takes OUT traffic next to IN
//! traffic, or a control transfer next to bulk traffic. `OUT_WINDOW` in
//! `device.rs` can rule out the first case at the cost of OUT throughput.
//!
//! [`host::HostDriver`] is an `embassy-usb` host driver for one SuperSpeed
//! device. It needs the `embassy` feature.
//!
//! The block has its own 125 MHz PLL, PIPE/UTMI clocks and link state machine,
//! separate from `usbhs`.

use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

use crate::pac::rcc::vals::UsbsspllRefsel;
use crate::pac::usbss::UsbssDevice;
#[cfg(feature = "embassy")]
use crate::pac::usbss::UsbssHost;
use crate::{interrupt, PeripheralType, RccPeripheral};

mod control;
mod device;
mod endpoint;
#[cfg(feature = "embassy")]
pub mod host;
mod regs;

pub use control::ControlPipe;
pub use device::{Bus, Driver, Error};
pub use endpoint::Endpoint;

pub use crate::pac::usbss::vals::LinkState;

/// Operating mode, set when a driver starts. Both modes share the interrupt
/// handlers and register block.
const MODE_DEVICE: u8 = 0;
const MODE_HOST: u8 = 1;

const LMP_SUBTYPE_MASK: u32 = 0xf << 5;
const LMP_SET_LINK_FUNC: u32 = 0x1 << 5;
const LMP_U2_INACT_TOUT: u32 = 0x2 << 5;
const LMP_PORT_CAP: u32 = 0x4 << 5;
const LMP_PORT_CFG: u32 = 0x5 << 5;
const LMP_PORT_CFG_RES: u32 = 0x6 << 5;
const LMP_LINK_SPEED: u32 = 1 << 9;
const NUM_HP_BUF: u32 = 4;
const UP_STREAM: u32 = 2 << 16;

struct State {
    mode: AtomicU8,
    device: device::State,
    #[cfg(feature = "embassy")]
    host: host::State,
}

impl State {
    const fn new(device: UsbssDevice) -> Self {
        Self {
            mode: AtomicU8::new(MODE_DEVICE),
            device: device::State::new(regs::Regs(device)),
            #[cfg(feature = "embassy")]
            host: host::State::new(),
        }
    }
}

trait SealedInstance: RccPeripheral {
    fn device() -> UsbssDevice;
    #[cfg(feature = "embassy")]
    fn host() -> UsbssHost;
    fn state() -> &'static State;
}

/// USBSS peripheral instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {
    /// Transaction / endpoint interrupt.
    type Interrupt: interrupt::typelevel::Interrupt;
    /// LINK state-machine interrupt.
    type LinkInterrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (usbss, $inst:ident) => {
        use crate::peripherals;

        impl SealedInstance for peripherals::$inst {
            fn device() -> UsbssDevice {
                unsafe { UsbssDevice::from_ptr(crate::pac::$inst.as_ptr()) }
            }

            #[cfg(feature = "embassy")]
            fn host() -> UsbssHost {
                unsafe { UsbssHost::from_ptr(crate::pac::$inst.as_ptr()) }
            }

            fn state() -> &'static State {
                static STATE: State = State::new(unsafe { UsbssDevice::from_ptr(crate::pac::$inst.as_ptr()) });
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
            type LinkInterrupt = crate::_generated::peripheral_interrupts::$inst::LINK;
        }
    };
);

/// GLOBAL interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        if T::state().mode.load(Ordering::Relaxed) == MODE_HOST {
            #[cfg(feature = "embassy")]
            host::on_interrupt::<T>();
        } else {
            device::on_interrupt::<T>();
        }
    }
}

/// LINK interrupt handler.
pub struct LinkInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::LinkInterrupt> for LinkInterruptHandler<T> {
    unsafe fn on_interrupt() {
        if T::state().mode.load(Ordering::Relaxed) == MODE_HOST {
            #[cfg(feature = "embassy")]
            host::on_link_interrupt::<T>();
        } else {
            device::on_interrupt::<T>();
        }
    }
}

/// Enables the USBSS PLL, the block clock, the PIPE clock and the UTMI clock.
unsafe fn init_clocks<T: Instance>() {
    crate::rcc::enable_usbss_pll(UsbsspllRefsel::F25MHZ);
    T::enable_and_reset();
    crate::pac::RCC.cfgr0().modify(|w| {
        w.set_pipeon(true);
        w.set_utmion(true);
    });
}

fn delay_cycles(count: u32) {
    for _ in 0..count {
        unsafe { core::arch::asm!("nop") };
    }
}
