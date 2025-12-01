#![no_std]

pub mod music;

use rp235x_hal::dma::Word;
use rp235x_hal::pio::{
    InstalledProgram, PIOExt, Rx, StateMachine, StateMachineIndex, Stopped, Tx, UninitStateMachine
};
use rp235x_hal as hal;

pub const OUTPUT_HZ: u32 = 1_000_000;
pub const PROGRAM_STEP_HZ: u32 = OUTPUT_HZ * 2;

pub const N_DAC_BITS: u8 = 12;
pub const DAC_MAX: u32 = (1 << N_DAC_BITS) - 1;

pub fn dac_program() -> pio::Program<32> {
    let mut a = pio::Assembler::<32>::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    // 2 step
    a.bind(&mut wrap_target);
    a.pull(false, true);
    a.out(pio::OutDestination::PINS, 32);
    a.bind(&mut wrap_source);

    a.assemble_with_wrap(wrap_source, wrap_target)
}

pub fn setup_dac<P, S>(
    base_pin_id: u8, 
    sys_clk_hz: u32, 
    installed: InstalledProgram<P>,
    sm: UninitStateMachine<(P, S)>
) -> (
    StateMachine<(P, S), Stopped>, Rx<(P, S), Word>, Tx<(P, S), Word>
)
where P: PIOExt, S: StateMachineIndex
{
    let int = (sys_clk_hz / PROGRAM_STEP_HZ) as u16;
    let rem = sys_clk_hz % PROGRAM_STEP_HZ;
    let frac = ((rem * 256) / PROGRAM_STEP_HZ) as u8;
    let (mut sm, rx, tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .out_pins(base_pin_id, N_DAC_BITS)
        .clock_divisor_fixed_point(int, frac)
        .build(sm);
    sm.set_pindirs((0..DAC_MAX as u8).map(|pin| (pin, hal::pio::PinDir::Output)));

    (sm, rx, tx)
}