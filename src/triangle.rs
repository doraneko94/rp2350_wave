#![no_std]
#![no_main]

use panic_halt as _;

use rp235x_hal::clocks::ClockSource;
use rp235x_hal as hal;

use hal::gpio::{FunctionPio0, Pin};
use hal::pio::PIOExt;
use hal::Sio;
use hal::watchdog::Watchdog;
use embedded_hal::digital::OutputPin;
const XTAL_FREQ_HZ: u32 = 12_000_000;
const OUTPUT_HZ: u32 = 1_000_000;
const PROGRAM_STEP_HZ: u32 = OUTPUT_HZ * 2;
const TRIANGLE_HZ: u32 = 1;
const OUTPUT_PER_TRIANGLE: u32 = OUTPUT_HZ / TRIANGLE_HZ;
const TRIANGLE_HALF: u32 = OUTPUT_PER_TRIANGLE / 2;
const COUNT_MAX: u32 = u32::MAX / OUTPUT_PER_TRIANGLE * OUTPUT_PER_TRIANGLE;

const N_DAC_BITS: u8 = 12;
const DAC_MAX: u32 = (1 << N_DAC_BITS) - 1;

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

    let mut a = pio::Assembler::<32>::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    // 2 step
    a.bind(&mut wrap_target);
    a.pull(false, true);
    a.out(pio::OutDestination::PINS, 32);
    a.bind(&mut wrap_source);

    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    
    let int = (hz / PROGRAM_STEP_HZ) as u16;
    let rem = hz % PROGRAM_STEP_HZ;
    let frac = ((rem * 256) / PROGRAM_STEP_HZ) as u8;
    let (mut sm, _, mut tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .out_pins(d0.id().num, N_DAC_BITS)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);
    sm.set_pindirs((0..DAC_MAX as u8).map(|pin| (pin, hal::pio::PinDir::Output)));
    sm.start();

    let mut led_pin = pins.gpio25.into_push_pull_output();
    led_pin.set_high().unwrap();

    let mut count: u32 = 0;
    loop {
        let tick = count % OUTPUT_PER_TRIANGLE;
        let value = if tick <= TRIANGLE_HALF {
            DAC_MAX * tick / TRIANGLE_HALF
        } else {
            2 * DAC_MAX - DAC_MAX * tick / TRIANGLE_HALF
        };
        let output = value.min(DAC_MAX);

        while tx.is_full() {}
        tx.write(output);
        
        count = count.wrapping_add(1);
        if count >= COUNT_MAX { count = 0; }
    }
}