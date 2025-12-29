#![no_std]
#![no_main]

use panic_halt as _;
use rp235x_hal::adc::AdcPin;
use rp235x_hal::dma::{DMAExt, SingleChannel, double_buffer};
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::peripheral::NVIC;
use embedded_hal::digital::OutputPin;

use rp235x_hal::{self as hal, singleton};
use hal::pac;
use pac::interrupt;

use hal::clocks::ClockSource;
use hal::gpio::{bank0, FunctionPio0, Pin, PullDown};
use hal::multicore::{Multicore, Stack};
use hal::pio::{InstalledProgram, PIOExt};
use hal::Sio;
use hal::watchdog::Watchdog;

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

const XTAL_FREQ_HZ: u32 = 12_000_000;
const OUTPUT_HZ: u32 = 1_000_000;
const PROGRAM_STEP_HZ: u32 = OUTPUT_HZ * 2;
const SQUARE_HZ: u32 = 1;
const OUTPUT_PER_SQUARE: u32 = OUTPUT_HZ / SQUARE_HZ;
const SQUARE_HALF: u32 = OUTPUT_PER_SQUARE / 2;
const COUNT_MAX: u32 = u32::MAX / OUTPUT_PER_SQUARE * OUTPUT_PER_SQUARE;

const N_DAC_BITS: u8 = 12;
const DAC_MAX: u32 = (1 << N_DAC_BITS) - 1;

const ADC_SAMPLES_PER_BUF: usize = 256;

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    let mut p = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(p.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        p.XOSC, 
        p.CLOCKS, 
        p.PLL_SYS, 
        p.PLL_USB, 
        &mut p.RESETS, 
        &mut watchdog
    ).ok().unwrap();
    let _sys_hz = clocks.system_clock.get_freq().to_Hz();

    let sio = Sio::new(p.SIO);
    let pins = hal::gpio::Pins::new(
        p.IO_BANK0,
        p.PADS_BANK0,
        sio.gpio_bank0,
        &mut p.RESETS,
    );

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        p.USB,
        p.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut p.RESETS
    ));
    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2E8A, 0x000A))
        .device_class(usbd_serial::USB_CLASS_CDC)
        .max_packet_size_0(64).expect("bad EP0 packet size")
        /*.strings(&[
            StringDescriptors::new(LangID::EN_GB)
                .manufacturer("Example")
                .product("Pico2 ADC DMA")
                .serial_number("0001")
        ]).expect("failed to set USB strings")*/
        .build();

    let mut led_pin = pins.gpio25.into_push_pull_output();
    led_pin.set_high().unwrap();

    loop {
        let _ = usb_dev.poll(&mut [&mut serial]);
    }
}