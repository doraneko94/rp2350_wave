#![no_std]
#![no_main]

use panic_halt as _;
use rp235x_hal as hal;

use hal::prelude::*;
use hal::{pac, sio::Sio, timer::Timer, watchdog::Watchdog};

use hal::gpio::{FunctionPio0, Pin};
use hal::pio::PIOExt;
use pio::Assembler;

const XTAL_FREQ_HZ: u32 = 12_000_000;
const OUTPUT_HZ: u32 = 1_000_000;

const DAC_BITS: u8 = 12;
const DAC_MAX: u32 = (1 << DAC_BITS) - 1;

const HALF: u32 = OUTPUT_HZ / 2;
const STEP: f64 = DAC_MAX as f64 / (HALF - 1) as f64;

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

fn triangle_sample(phase: u32) -> u16 {
    let half = OUTPUT_HZ / 2;

    let value = if phase < half {
        (phase as f64) * STEP
    } else {
        let down_phase = (phase - half) as f64;
        DAC_MAX as f64 - down_phase * STEP
    };
    
    value as u16
}

#[hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
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

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0, 
        pac.PADS_BANK0, 
        sio.gpio_bank0,
        &mut pac.RESETS
    );

    let d0: Pin<hal::gpio::bank0::Gpio0, FunctionPio0, hal::gpio::PullDown> = pins.gpio0.into_push_pull_output().into_function();
    let _d1: Pin<hal::gpio::bank0::Gpio1, FunctionPio0, hal::gpio::PullDown> = pins.gpio1.into_push_pull_output().into_function();
    let _d2: Pin<hal::gpio::bank0::Gpio2, FunctionPio0, hal::gpio::PullDown> = pins.gpio2.into_push_pull_output().into_function();
    let _d3: Pin<hal::gpio::bank0::Gpio3, FunctionPio0, hal::gpio::PullDown> = pins.gpio3.into_push_pull_output().into_function();
    let _d4: Pin<hal::gpio::bank0::Gpio4, FunctionPio0, hal::gpio::PullDown> = pins.gpio4.into_push_pull_output().into_function();
    let _d5: Pin<hal::gpio::bank0::Gpio5, FunctionPio0, hal::gpio::PullDown> = pins.gpio5.into_push_pull_output().into_function();
    let _d6: Pin<hal::gpio::bank0::Gpio6, FunctionPio0, hal::gpio::PullDown> = pins.gpio6.into_push_pull_output().into_function();
    let _d7: Pin<hal::gpio::bank0::Gpio7, FunctionPio0, hal::gpio::PullDown> = pins.gpio7.into_push_pull_output().into_function();
    let _d8: Pin<hal::gpio::bank0::Gpio8, FunctionPio0, hal::gpio::PullDown> = pins.gpio8.into_push_pull_output().into_function();
    let _d9: Pin<hal::gpio::bank0::Gpio9, FunctionPio0, hal::gpio::PullDown> = pins.gpio9.into_push_pull_output().into_function();
    let _d10: Pin<hal::gpio::bank0::Gpio10, FunctionPio0, hal::gpio::PullDown> = pins.gpio10.into_push_pull_output().into_function();
    let _d11: Pin<hal::gpio::bank0::Gpio11, FunctionPio0, hal::gpio::PullDown> = pins.gpio11.into_push_pull_output().into_function();

    let mut a = Assembler::<32>::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    a.set(pio::SetDestination::PINDIRS, 1);
    a.bind(&mut wrap_target);
    a.pull(true, false);
    a.out(pio::OutDestination::PINS, DAC_BITS);
    a.bind(&mut wrap_source);
    
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    
    let program = a.assemble_with_wrap(wrap_source, wrap_target);
    let installed = pio0.install(&program).unwrap();

    let sys_hs = clocks.system_clock.freq().to_Hz();
    let target_instr = OUTPUT_HZ * 2;
    let int = (sys_hs / target_instr) as u16;
    let rem = sys_hs % target_instr;
    let frac = ((rem * 256) / target_instr) as u8;

    let (mut sm, _, mut tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .set_pins(d0.id().num, DAC_BITS as u8)
        .out_pins(d0.id().num, DAC_BITS as u8)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);
    
    sm.set_pindirs([(d0.id().num, hal::pio::PinDir::Output); 12]);
    sm.start();

    let timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let ticks_per_sec: u64 = 1_000_000;

    let t0 = timer.get_counter().ticks();

    let mut produced: u64 = 0;
    
    loop {
        let now = timer.get_counter().ticks();
        let elapsed_ticks = now.wrapping_sub(t0);
        let elapsed_us: u64 = elapsed_ticks;

        /*let consumed: u64 = elapsed_us * (OUTPUT_HZ as u64) / ticks_per_sec;

        while produced.saturating_sub(consumed) > 16u64 {
            let now2 = timer.get_counter().ticks();
            let elapsed_us2 = now2.wrapping_sub(t0);
            let consumed2 = elapsed_us2 * (OUTPUT_HZ as u64) / ticks_per_sec;

            if produced.saturating_sub(consumed2) <= 16u64 { break; }
        }

        if consumed.saturating_sub(produced) > 0u64 {
            panic!("CPU too slow for waveform generation");
        }

        let phase_in_period: u32 = (produced % (OUTPUT_HZ as u64)) as u32;
        let sample = triangle_sample(phase_in_period);

        while tx.is_full() {}

        tx.write(sample as u32);

        produced += 1;*/
        if elapsed_us / 500_000 % 2 == 0 {
            tx.write(DAC_MAX);
        } else {
            tx.write(0);
        }
    }
}
