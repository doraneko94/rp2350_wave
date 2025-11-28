#![no_std]
#![no_main]

use panic_halt as _;

use rp235x_hal::clocks::ClockSource;
use rp235x_hal as hal;

use hal::gpio::{FunctionPio0, Pin};
use hal::pio::PIOExt;
use hal::Sio;
use hal::watchdog::Watchdog;

use rp2350_wave::{DAC_MAX, dac_program, setup_dac};

const XTAL_FREQ_HZ: u32 = 12_000_000;

// sin(2 * pi * 100kHz) / OUTPUT_HZ
const SIN_DELTA: f64 = 0.5877852522924731;
// cos(2 * pi * 100kHz) / OUTPUT_HZ
const COS_DELTA: f64 = 0.8090169943749475;

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC, 
        pac.CLOCKS, 
        pac.PLL_SYS, 
        pac.PLL_USB, 
        &mut pac.RESETS, 
        &mut watchdog
    ).ok().unwrap();
    let hz = clocks.system_clock.get_freq().to_Hz();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let d0: Pin<_, FunctionPio0, _> = pins.gpio0.into_function();
    let _d1: Pin<_, FunctionPio0, _> = pins.gpio1.into_function();
    let _d2: Pin<_, FunctionPio0, _> = pins.gpio2.into_function();
    let _d3: Pin<_, FunctionPio0, _> = pins.gpio3.into_function();
    let _d4: Pin<_, FunctionPio0, _> = pins.gpio4.into_function();
    let _d5: Pin<_, FunctionPio0, _> = pins.gpio5.into_function();
    let _d6: Pin<_, FunctionPio0, _> = pins.gpio6.into_function();
    let _d7: Pin<_, FunctionPio0, _> = pins.gpio7.into_function();
    let _d8: Pin<_, FunctionPio0, _> = pins.gpio8.into_function();
    let _d9: Pin<_, FunctionPio0, _> = pins.gpio9.into_function();
    let _d10: Pin<_, FunctionPio0, _> = pins.gpio10.into_function();
    let _d11: Pin<_, FunctionPio0, _> = pins.gpio11.into_function();

    let program = dac_program();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    
    let (sm, _, mut tx) = setup_dac(d0.id().num, hz, installed, sm0);
    sm.start();

    let (mut sn, mut cn) = (0f64, 1f64);
    loop {
        let s = sn * COS_DELTA + cn * SIN_DELTA;
        let c = cn * COS_DELTA - sn * SIN_DELTA;
        let s_shift = (s + 1.0).max(0.0) / 2.0;
        let output_f64 = DAC_MAX as f64 * s_shift;
        let output = (output_f64 as u32).min(DAC_MAX);

        while tx.is_full() {}
        tx.write(output);
        
        (sn, cn) = (s, c);
    }
}