#![no_std]

use rp235x_hal::dma::Word;
use rp235x_hal::pio::{
    InstalledProgram, PIOExt, Rx, StateMachine, StateMachineIndex, Stopped, Tx, UninitStateMachine, ValidStateMachine
};
use rp235x_hal as hal;

use hal::gpio::{FunctionPio0, Pin, Pins, PullDown};
use hal::gpio::bank0::{
    Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5,
    Gpio6, Gpio7, Gpio8, Gpio9, Gpio10, Gpio11
};

pub const OUTPUT_HZ: u32 = 1_000_000;
pub const PROGRAM_STEP_HZ: u32 = OUTPUT_HZ * 2;

pub const N_DAC_BITS: u8 = 12;
pub const DAC_MAX: u32 = (1 << N_DAC_BITS) - 1;

pub struct Dac {
    pub pins: DacPins
}

pub struct DacPins {
    pub d0: Pin<Gpio0, FunctionPio0, PullDown>,
    pub d1: Pin<Gpio1, FunctionPio0, PullDown>,
    pub d2: Pin<Gpio2, FunctionPio0, PullDown>,
    pub d3: Pin<Gpio3, FunctionPio0, PullDown>,
    pub d4: Pin<Gpio4, FunctionPio0, PullDown>,
    pub d5: Pin<Gpio5, FunctionPio0, PullDown>,
    pub d6: Pin<Gpio6, FunctionPio0, PullDown>,
    pub d7: Pin<Gpio7, FunctionPio0, PullDown>,
    pub d8: Pin<Gpio8, FunctionPio0, PullDown>,
    pub d9: Pin<Gpio9, FunctionPio0, PullDown>,
    pub d10: Pin<Gpio10, FunctionPio0, PullDown>,
    pub d11: Pin<Gpio11, FunctionPio0, PullDown>,
}

impl DacPins {
    /*pub fn new(pins: &mut Pins) -> Self {
        let d0: Pin<_, FunctionPio0, _> = pins.gpio0.into_function();
        let d1: Pin<_, FunctionPio0, _> = pins.gpio1.into_function();
        let d2: Pin<_, FunctionPio0, _> = pins.gpio2.into_function();
        let d3: Pin<_, FunctionPio0, _> = pins.gpio3.into_function();
        let d4: Pin<_, FunctionPio0, _> = pins.gpio4.into_function();
        let d5: Pin<_, FunctionPio0, _> = pins.gpio5.into_function();
        let d6: Pin<_, FunctionPio0, _> = pins.gpio6.into_function();
        let d7: Pin<_, FunctionPio0, _> = pins.gpio7.into_function();
        let d8: Pin<_, FunctionPio0, _> = pins.gpio8.into_function();
        let d9: Pin<_, FunctionPio0, _> = pins.gpio9.into_function();
        let d10: Pin<_, FunctionPio0, _> = pins.gpio10.into_function();
        let d11: Pin<_, FunctionPio0, _> = pins.gpio11.into_function();

        Self {
            d0, d1, d2, d3, d4, d5,
            d6, d7, d8, d9, d10, d11
        }
    }*/
}

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
    //sm.start();
    (sm, rx, tx)
}