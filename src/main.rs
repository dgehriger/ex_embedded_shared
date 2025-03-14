#![no_std]
#![no_main]

use core::cell::RefCell;

use arduino_mkrzero as bsp;
use bsp::hal;

use embassy_sync::{blocking_mutex, once_lock};

#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use bsp::entry;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{interrupt, CorePeripherals, Peripherals};
use hal::prelude::*;

use hal::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use cortex_m::peripheral::NVIC;

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;

// Declare alias for a mutex-protected shared object (`Mutex`) with interior mutability (`RefCell`)
type SharedMutableType<T> = blocking_mutex::Mutex<blocking_mutex::raw::NoopRawMutex, RefCell<T>>;

// Declare a global static variable for the USB serial port
// This is a `OnceLock` because we cannot statically initialize the serial port, but need to initialize it at runtime
static mut USB_SERIAL: once_lock::OnceLock<SharedMutableType<SerialPort<UsbBus>>> = once_lock::OnceLock::new();

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let pins = bsp::pins::Pins::new(peripherals.PORT);
    let mut led = bsp::pin_alias!(pins.led).into_push_pull_output();
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let usb_n = bsp::pin_alias!(pins.usb_n);
    let usb_p = bsp::pin_alias!(pins.usb_p);

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(bsp::usb::usb_allocator(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.PM,
            usb_n.into(),
            usb_p.into(),
        ));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    unsafe {
        USB_SERIAL.init(blocking_mutex::Mutex::new(RefCell::new(SerialPort::new(bus_allocator))));

        USB_BUS = Some(
            UsbDeviceBuilder::new(bus_allocator, UsbVidPid(0x2222, 0x3333))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build(),
        );
    }

    unsafe {
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    loop {
        delay.delay_ms(200u8);
        led.set_high().unwrap();
        delay.delay_ms(200u8);
        led.set_low().unwrap();

        // Turn off interrupts so we don't fight with the interrupt
        cortex_m::interrupt::free(|_| unsafe {
            // Get the serial port (if initialized)
            if let Some(serial) = USB_SERIAL.try_get() {
                // Get unique access...
                serial.lock(|serial| {
                    // ...and mutabily borrow the serial port
                    let mut serial = serial.borrow_mut();
                    serial.write("log line\r\n".as_bytes());
                });
            }
        });
    }
}

fn poll_usb() {
    unsafe {
        if let Some(usb_dev) = USB_BUS.as_mut() {
            // Get the serial port (if initialized)
            if let Some(serial) = USB_SERIAL.try_get() {
                // Get unique access...
                serial.lock(|serial| {
                    // ...and mutabily borrow the serial port
                    let mut serial = serial.borrow_mut();

                    // Need to use `&mut *serial` because `serial` is a wrapper type
                    usb_dev.poll(&mut [&mut *serial]);

                    // Make the other side happy
                    let mut buf = [0u8; 16];
                    let _ = serial.read(&mut buf);
                });
            }
        
        }
    };
}

#[interrupt]
#[allow(non_snake_case)]
fn USB() {
    poll_usb();
}