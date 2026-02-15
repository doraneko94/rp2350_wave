#![no_std]
#![no_main]

use panic_halt as _;
use rp235x_hal::gpio::FunctionPio1;

use core::cell::{RefCell, UnsafeCell};
use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m::peripheral::NVIC;
use critical_section::Mutex;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use heapless::spsc::Queue;

use rp235x_hal as hal;
use hal::pac;
use pac::interrupt;

use hal::clocks::ClockSource;
use rp235x_hal::dma::{DMAExt, double_buffer, SingleChannel};
use hal::gpio::{bank0, FunctionPio0, Pin, PullDown};
use hal::multicore::{Multicore, Stack};
use hal::pio::{InstalledProgram, PIOExt};
use hal::Sio;
use hal::watchdog::Watchdog;

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

const XTAL_FREQ_HZ: u32 = 12_000_000;
const DIV_INT_ULTRASONIC: u16 = 10;
const DIV_INT_LOW_POWER: u16 = 75;

const OUTPUT_HZ: u32 = 1_000_000;
const PROGRAM_STEP_HZ: u32 = OUTPUT_HZ * 2;
const SQUARE_HZ: u32 = 20_000;
const OUTPUT_PER_SQUARE: u32 = OUTPUT_HZ / SQUARE_HZ;
const SQUARE_HALF: u32 = OUTPUT_PER_SQUARE / 2;
const COUNT_MAX: u32 = u32::MAX / OUTPUT_PER_SQUARE * OUTPUT_PER_SQUARE;
const DAC_MAX: u32 = 65535;

const WORDS_PER_BUF: usize = 2048;    // recommend >= 2048 to reduce IRQ rate
const TOTAL_BYTES: usize = WORDS_PER_BUF * 4;
const N_BUFS: usize = 8;                    // >= 6..12 recommended for burst absorption

const TX_CHUNK: usize = 384;

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

static CORE1_STACK: Stack<4096> = Stack::new();

static DMA_DONE: AtomicBool = AtomicBool::new(false);

struct PdmPool(UnsafeCell<[[u32; WORDS_PER_BUF]; N_BUFS]>);
unsafe impl Sync for PdmPool {} // we will enforce safety via our own index discipline

static PDM_POOL: PdmPool = PdmPool(UnsafeCell::new([[0; WORDS_PER_BUF]; N_BUFS]));

// Indices queues
static FREE_Q: Mutex<RefCell<Queue<usize, N_BUFS>>> = Mutex::new(RefCell::new(Queue::new()));
static READY_Q: Mutex<RefCell<Queue<usize, N_BUFS>>> = Mutex::new(RefCell::new(Queue::new()));

// SAFETY: caller must ensure no aliasing of the same idx (our DMA/queue discipline does that)
#[inline(always)]
unsafe fn pool_buf_mut(idx: usize) -> &'static mut [u32; WORDS_PER_BUF] {
    unsafe { &mut (*PDM_POOL.0.get())[idx] }
}
// SAFETY: caller must ensure this buffer is not being mutated concurrently (READY_Q buffers are not in DMA)
#[inline(always)]
unsafe fn pool_buf_ref(idx: usize) -> &'static [u32; WORDS_PER_BUF] {
    unsafe { &(*PDM_POOL.0.get())[idx] }
}

#[allow(non_snake_case)]
#[cortex_m_rt::interrupt]
fn DMA_IRQ_0() {
    DMA_DONE.store(true, core::sync::atomic::Ordering::Release);

    let dma = unsafe { &*pac::DMA::ptr() };
    unsafe {
        // ch0 + ch1
        dma.ints0().write(|w| w.bits((1 << 0) | (1 << 1)));
    }
}

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
    let sys_hz = clocks.system_clock.get_freq().to_Hz();

    let mut timer = hal::Timer::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);

    let mut sio = Sio::new(p.SIO);
    let pins = hal::gpio::Pins::new(
        p.IO_BANK0,
        p.PADS_BANK0,
        sio.gpio_bank0,
        &mut p.RESETS,
    );

    let pio_pins = (
        pins.gpio0.into_function::<FunctionPio0>(),
        pins.gpio1.into_function::<FunctionPio0>(),
        pins.gpio2.into_function::<FunctionPio0>(),
        pins.gpio3.into_function::<FunctionPio0>(),
        pins.gpio4.into_function::<FunctionPio0>(),
        pins.gpio5.into_function::<FunctionPio0>(),
        pins.gpio6.into_function::<FunctionPio0>(),
        pins.gpio7.into_function::<FunctionPio0>(),
        pins.gpio8.into_function::<FunctionPio0>(),
        pins.gpio9.into_function::<FunctionPio0>(),
        pins.gpio10.into_function::<FunctionPio0>(),
        pins.gpio11.into_function::<FunctionPio0>(),
        pins.gpio12.into_function::<FunctionPio0>(),
        pins.gpio13.into_function::<FunctionPio0>(),
        pins.gpio14.into_function::<FunctionPio0>(),
        pins.gpio15.into_function::<FunctionPio0>(),
    );

    let mut a = pio::Assembler::<32>::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    // 2 step
    a.bind(&mut wrap_target);
    a.pull(false, true);
    a.out(pio::OutDestination::PINS, 16);
    a.bind(&mut wrap_source);

    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    let (mut pio0, sm0, _, _, _) = p.PIO0.split(&mut p.RESETS);
    let installed = pio0.install(&program).unwrap();

    let mut mc = Multicore::new(&mut p.PSM, &mut p.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    core1
        .spawn(CORE1_STACK.take().unwrap(), move || {
            core1_pio_task(sm0, installed, pio_pins, sys_hz)
        })
        .unwrap();

    let dat_pin = pins.gpio16.into_function::<FunctionPio1>();
    let clk_pin = pins.gpio17.into_function::<FunctionPio1>();
    let mut vdd_pin = pins.gpio18.into_push_pull_output();
    let _ = vdd_pin.set_low();

    let side_set = pio::SideSet::new(false, 1, false);
    let mut a = pio::Assembler::<32>::new_with_side_set(side_set);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    let delay: u8 = 1;
    // 4 cycle
    a.bind(&mut wrap_target);
    a.in_with_delay_and_side_set(pio::InSource::PINS, 1, delay, 1);
    a.nop_with_delay_and_side_set(delay, 0);
    a.bind(&mut wrap_source);

    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    let mut led_pin = pins.gpio25.into_push_pull_output();
    let _ = led_pin.set_high().ok();

    let (mut pio1, sm0_pio1, _, _, _) = p.PIO1.split(&mut p.RESETS);
    let installed = pio1.install(&program).unwrap();

    let (mut sm_pio1, rx_pio1, _) = hal::pio::PIOBuilder::from_installed_program(installed)
        .side_set_pin_base(clk_pin.id().num)
        .in_pin_base(dat_pin.id().num)
        .clock_divisor_fixed_point(DIV_INT_LOW_POWER, 0)
        .autopush(true)
        .push_threshold(32)
        .in_shift_direction(hal::pio::ShiftDirection::Right)
        .build(sm0_pio1);
    sm_pio1.set_pindirs([(clk_pin.id().num as u8, hal::pio::PinDir::Output)]);

    let _ = vdd_pin.set_high();
    timer.delay_ms(60);

    let mut sm_pio1 = sm_pio1.start();
    timer.delay_ms(20);

    sm_pio1.set_clock_divisor(DIV_INT_ULTRASONIC as f32);
    timer.delay_ms(12);
    
    let mut dch = p.DMA.dyn_split(&mut p.RESETS);
    let mut ch0 = dch.ch0.take().unwrap();
    let mut ch1 = dch.ch1.take().unwrap();
    ch0.enable_irq0();
    ch1.enable_irq0();
    unsafe { NVIC::unmask(pac::Interrupt::DMA_IRQ_0) };

    // ====== Init free/ready queues ======
    critical_section::with(|cs| {
        let mut fq = FREE_Q.borrow_ref_mut(cs);
        for i in 0..N_BUFS {
            let _ = fq.enqueue(i);
        }
    });

    // Take 2 buffers for ping-pong DMA
    let (idx0, idx1) = critical_section::with(|cs| {
        let mut fq = FREE_Q.borrow_ref_mut(cs);
        (fq.dequeue().unwrap(), fq.dequeue().unwrap())
    });

    let buf0: &'static mut [u32; WORDS_PER_BUF] = unsafe { pool_buf_mut(idx0) };
    let buf1: &'static mut [u32; WORDS_PER_BUF] = unsafe { pool_buf_mut(idx1) };

    // Track which indices correspond to DMA ping/pong sequencing.
    // - dma_cur_idx: the one that just finished on wait()
    // - dma_next_idx: the one currently being filled (already queued)
    let mut dma_cur_idx: usize = idx0;
    let mut dma_next_idx: usize = idx1;

    let mut transfer = double_buffer::Config::new(
        (ch0, ch1), 
        rx_pio1,
        buf0
    ).start().write_next(buf1);

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
        .strings(&[
            StringDescriptors::new(LangID::EN_US) // EN_GBにすると死ぬ
                .manufacturer("Example")
                .product("SPJ0641 PDM Ultrasonic")
                .serial_number("0001")
        ]).expect("failed to set USB strings")
        .build();

    // let mut led_pin = pins.gpio25.into_push_pull_output();
    // let _ = led_pin.set_high().ok();

    // Current buffer being transmitted
    let mut cur_tx_idx: Option<usize> = None;
    let mut cur_byte_off: usize = 0;

    // Small packed TX staging (no full-buffer copy)
    let mut tx_chunk = [0u8; TX_CHUNK];
    let mut tx_chunk_len: usize = 0;
    let mut tx_chunk_off: usize = 0;

    let mut configured = false;

    loop {
        let _ = usb_dev.poll(&mut [&mut serial]);

        if !configured && usb_dev.state() == UsbDeviceState::Configured {
            configured = true;
            // ここで「必ず1回出す」
            let _ = serial.write(b"USB configured\r\n");
        }

        if !configured {
            continue;
        }

        // === 1) USB TX: if stagged chunk exists, flush it ===
        if tx_chunk_off < tx_chunk_len {
            match serial.write(&tx_chunk[tx_chunk_off..tx_chunk_len]) {
                Ok(0) | Err(_) => {}
                Ok(n) => {
                    tx_chunk_off += n;
                    if tx_chunk_off >= tx_chunk_len {
                        tx_chunk_off = 0;
                        tx_chunk_len = 0;
                    }
                }
            }
        } else {
            // === 2) No stagged chunk: build next packed chunk from ready buffers ===

            // If no current buffer, grab one from READY_Q
            if cur_tx_idx.is_none() {
                cur_tx_idx = critical_section::with(|cs| {
                    let mut rq = READY_Q.borrow_ref_mut(cs);
                    rq.dequeue()
                });
                cur_byte_off = 0;
            }

            if let Some(idx) = cur_tx_idx {
                let words = unsafe { pool_buf_ref(idx) };

                if cur_byte_off < TOTAL_BYTES {
                    let mut woff = cur_byte_off / 4;
                    let mut boff = cur_byte_off % 4;

                    let mut out_n = 0usize;

                    while out_n < TX_CHUNK && (woff < WORDS_PER_BUF) {
                        let le = words[woff].to_le_bytes();

                        while out_n < TX_CHUNK && boff < 4 {
                            tx_chunk[out_n] = le[boff];
                            out_n += 1;
                            boff += 1;
                        }

                        if boff >= 4 {
                            boff = 0;
                            woff += 1;
                        }
                    }

                    cur_byte_off += out_n;
                    tx_chunk_len = out_n;
                    tx_chunk_off = 0;
                } else {
                    critical_section::with(|cs| {
                        let mut fq = FREE_Q.borrow_ref_mut(cs);
                        let _ = fq.enqueue(idx);
                    });
                    cur_tx_idx = None;
                    cur_byte_off = 0;
                }
            }
        }

        // === 3) DMA completion handling ===
        if DMA_DONE.swap(false, Ordering::AcqRel) {
            let (_filled_buf, next) = transfer.wait();

            // The finished buffer index is known by sequencing, not by pointer probing.
            let filled_idx = dma_cur_idx;

            // After wait(), the "current" for the next cycle is what used to be dma_next_idx.
            dma_cur_idx = dma_next_idx;

            // Try to get a new buffer to schedule after the one currently being filled.
            let schedule_idx_opt = critical_section::with(|cs| {
                let mut fq = FREE_Q.borrow_ref_mut(cs);
                fq.dequeue()
            });

            let sched_buf = if let Some(schedule_idx) = schedule_idx_opt {
                // We have a fresh buffer to schedule, so we can keep filled_idx (if READY_Q has space).
                let pushed = critical_section::with(|cs| {
                    let mut rq = READY_Q.borrow_ref_mut(cs);
                    rq.enqueue(filled_idx).is_ok()
                });

                if !pushed {
                    critical_section::with(|cs| {
                        let mut fq = FREE_Q.borrow_ref_mut(cs);
                        let _ = fq.enqueue(filled_idx);
                    });
                }
                dma_next_idx = schedule_idx;
                unsafe { pool_buf_mut(schedule_idx) }
            } else {
                dma_next_idx = filled_idx;
                unsafe { pool_buf_mut(filled_idx) }
            };

            transfer = next.write_next(sched_buf);
        }
    }
}

fn core1_pio_task(
    sm0: hal::pio::UninitStateMachine<(hal::pac::PIO0, hal::pio::SM0)>,
    installed: InstalledProgram<hal::pac::PIO0>,
    pio_pins: (
        Pin<bank0::Gpio0, FunctionPio0, PullDown>,
        Pin<bank0::Gpio1, FunctionPio0, PullDown>,
        Pin<bank0::Gpio2, FunctionPio0, PullDown>,
        Pin<bank0::Gpio3, FunctionPio0, PullDown>,
        Pin<bank0::Gpio4, FunctionPio0, PullDown>,
        Pin<bank0::Gpio5, FunctionPio0, PullDown>,
        Pin<bank0::Gpio6, FunctionPio0, PullDown>,
        Pin<bank0::Gpio7, FunctionPio0, PullDown>,
        Pin<bank0::Gpio8, FunctionPio0, PullDown>,
        Pin<bank0::Gpio9, FunctionPio0, PullDown>,
        Pin<bank0::Gpio10, FunctionPio0, PullDown>,
        Pin<bank0::Gpio11, FunctionPio0, PullDown>,
        Pin<bank0::Gpio12, FunctionPio0, PullDown>,
        Pin<bank0::Gpio13, FunctionPio0, PullDown>,
        Pin<bank0::Gpio14, FunctionPio0, PullDown>,
        Pin<bank0::Gpio15, FunctionPio0, PullDown>,
    ),
    sys_hz: u32
) -> ! {
    let d0 = pio_pins.0;
    let _d1 = pio_pins.1;
    let _d2 = pio_pins.2;
    let _d3 = pio_pins.3;
    let _d4 = pio_pins.4;
    let _d5 = pio_pins.5;
    let _d6 = pio_pins.6;
    let _d7 = pio_pins.7;
    let _d8 = pio_pins.8;
    let _d9 = pio_pins.9;
    let _d10 = pio_pins.10;
    let _d11 = pio_pins.11;
    let _d12 = pio_pins.12;
    let _d13 = pio_pins.13;
    let _d14 = pio_pins.14;
    let _d15 = pio_pins.15;

    let int = (sys_hz / PROGRAM_STEP_HZ) as u16;
    let rem = sys_hz % PROGRAM_STEP_HZ;
    let frac = ((rem * 256) / PROGRAM_STEP_HZ) as u8;
    let (mut sm, _, mut tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .out_pins(d0.id().num, 16)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);
    sm.set_pindirs((0..16 as u8).map(|pin| (pin, hal::pio::PinDir::Output)));
    sm.start();

    let mut count: u32 = 0;
    loop {
        let output = if (count / SQUARE_HALF) % 2 == 0 { DAC_MAX } else { 0 };

        while tx.is_full() {}
        tx.write(output);
        
        count = count.wrapping_add(1);
        if count >= COUNT_MAX { count = 0; }
    }
}