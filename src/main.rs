#![no_std]
#![no_main]

#[allow(unused_imports)]
use panic_halt as _;
use rp2040_hal as hal;
use hal::pac;
use embedded_hal::digital::v2::OutputPin;
use usb_device::class_prelude::*;
use usbd_serial::SerialPort;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid, StringDescriptors};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;
const BLINK_DELAY: u32 = 100_000;


#[rp2040_hal::entry]
fn main() -> ! {
    // Grab singleton peripherals
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up watchdog and clocks
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(XTAL_FREQ_HZ, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB, &mut pac.RESETS, &mut watchdog).ok().unwrap();

    // Configure GPIO pins
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let mut led = pins.gpio25.into_push_pull_output();

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(pac.USBCTRL_REGS, pac.USBCTRL_DPRAM, clocks.usb_clock, true, &mut pac.RESETS));

    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default().manufacturer("Your Manufacturer").product("Your Product").serial_number("1234567890")])
        .unwrap()
        .device_class(0x02) // CDC class
        .build();

    loop {
        led.set_high().unwrap();  // LED on
        let _ = serial.write(b"LED on!\r\n");

        // Wait for a short time
        for _ in 0..BLINK_DELAY {
            usb_dev.poll(&mut [&mut serial]);
        }

        led.set_low().unwrap();
        let _ = serial.write(b"LED off!\r\n");

        // Wait for a short time
        for _ in 0..BLINK_DELAY {
            usb_dev.poll(&mut [&mut serial]);
        }
    }
}
