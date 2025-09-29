#![no_std]
#![no_main]
extern crate panic_halt;
extern crate embedded_hal;
extern crate rp2040_hal;
extern crate cortex_m;

use panic_halt as _;  // Ensure we halt the program on panic (if we don't mention this crate it won't be linked)
use rp2040_hal as hal;  // Alias for our HAL crate
use hal::pac;  // A shorter alias for the Peripheral Access Crate, which provides low-level register access
use embedded_hal::digital::v2::OutputPin;  // Some traits we need
use rp2040_hal::clocks::Clock;  // Some traits we need
use usb_device::class_prelude::*;
use usbd_serial::SerialPort;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid, StringDescriptors};


/// Boot block placed at the start of the program image for the ROM bootloader.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

const BLINK_DELAY: u32 = 100_000;


#[rp2040_hal::entry]
fn main() -> ! {
    // 1. Grab singleton peripherals
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // 2. Set up watchdog and clocks
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB, &mut pac.RESETS, &mut watchdog
    ).ok().unwrap();

    // 3. Set up delay timer
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // 4. Configure GPIO pins
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let mut led = pins.gpio25.into_push_pull_output();

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Tiny delay before enabling USB
    delay.delay_ms(100); // 100ms usually enough

    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[
            StringDescriptors::default()
                .manufacturer("Your Manufacturer")
                .product("Your Product")
                .serial_number("1234567890"),
        ])
        .unwrap()
        .device_class(0x02) // CDC class
        .build();

    loop {
        led.set_high().unwrap();  // LED on
        for _ in 0..BLINK_DELAY {
            // Poll keeps the USB responsive
            usb_dev.poll(&mut [&mut serial]);
        }
        let _ = serial.write(b"LED on!\r\n");

        led.set_low().unwrap();   // LED off
        for _ in 0..BLINK_DELAY {
            // Poll keeps the USB responsive
            usb_dev.poll(&mut [&mut serial]);
        }
        let _ = serial.write(b"LED off!\r\n");
    }
}
