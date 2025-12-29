#![no_std]
#![no_main]

use panic_halt as _;

use rp235x_hal as hal;

use embedded_hal::digital::OutputPin;

use hal::dma::{self, DMAExt};
use hal::usb::UsbBus;

use usb_device::bus::UsbBusAllocator;
use usb_device::descriptor::lang_id::LangID;
use usb_device::device::StringDescriptors;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

const XTAL_FREQ_HZ: u32 = 12_000_000;
const SAMPLES_PER_BUF: usize = 256;

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    // ========= PAC / clocks =========
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // ========= GPIO =========
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led = pins.gpio25.into_push_pull_output();
    let mut led_on = false;

    // ========= USB =========
    let usb_bus: &'static UsbBusAllocator<UsbBus> = hal::singleton!(
        : UsbBusAllocator<UsbBus> =
            UsbBusAllocator::new(
                UsbBus::new(
                    pac.USB,
                    pac.USB_DPRAM,
                    clocks.usb_clock,
                    true, // force_vbus_detect
                    &mut pac.RESETS,
                )
            )
    )
    .unwrap();

    let mut serial = SerialPort::new(usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0xCAFE, 0x4001))
        .device_class(usbd_serial::USB_CLASS_CDC)
        .max_packet_size_0(64).expect("bad EP0 packet size")
        .strings(&[
            StringDescriptors::new(LangID::EN_US)
                .manufacturer("Example")
                .product("Pico2 ADC DMA")
                .serial_number("0001")
        ]).expect("failed to set USB strings")
        .build();

    // ========= ADC =========
    let mut adc = hal::adc::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin = hal::adc::AdcPin::new(pins.gpio26.into_floating_input()).unwrap();

    let mut fifo = adc
        .build_fifo()
        .clock_divider(0, 0)
        .set_channel(&mut adc_pin)
        .enable_dma()
        .start_paused(); // paused

    // ========= DMA =========
    let mut dch = pac.DMA.dyn_split(&mut pac.RESETS);
    let ch0 = dch.ch0.take().unwrap();
    let ch1 = dch.ch1.take().unwrap();

    let buf_a = hal::singleton!(: [u16; SAMPLES_PER_BUF] = [0; SAMPLES_PER_BUF]).unwrap();
    let buf_b = hal::singleton!(: [u16; SAMPLES_PER_BUF] = [0; SAMPLES_PER_BUF]).unwrap();

    let mut transfer = dma::double_buffer::Config::new(
        (ch0, ch1),
        fifo.dma_read_target(),
        buf_a,
    )
    .start()
    .write_next(buf_b);

    // DMA開始後にADC開始
    fifo.resume();

    // ========= framing =========
    // frame: [u32 seq][u16 n][u16 samples...]
    let mut seq: u32 = 0;
    let mut frame = [0u8; 6 + SAMPLES_PER_BUF * 2];

    loop {
        // まずUSBスタックを回す（列挙・詰まり対策）
        let _ = usb_dev.poll(&mut [&mut serial]);

        // DMA完了待ち（どちらかのバッファが埋まる）
        let (filled, next) = transfer.wait();

        // ---- 重要：filledの中身を“先にframeへコピー”してからDMAへ返す ----
        frame[0..4].copy_from_slice(&seq.to_le_bytes());
        frame[4..6].copy_from_slice(&(SAMPLES_PER_BUF as u16).to_le_bytes());

        let mut off = 6;
        for &s in filled.iter() {
            let b = s.to_le_bytes();
            frame[off] = b[0];
            frame[off + 1] = b[1];
            off += 2;
        }

        // ここで filled の参照はもう使わないので、すぐDMAへ返却して取り込み継続
        transfer = next.write_next(filled);

        // heartbeat（toggle使わず）
        if (seq & 0x3F) == 0 {
            led_on = !led_on;
            if led_on { let _ = led.set_high(); } else { let _ = led.set_low(); }
        }

        // ---- USB送信（frameはDMAと無関係な独立バッファ）----
        let mut data = &frame[..off];
        while !data.is_empty() {
            let _ = usb_dev.poll(&mut [&mut serial]);
            match serial.write(data) {
                Ok(0) => {}
                Ok(n) => data = &data[n..],
                Err(UsbError::WouldBlock) => {}
                Err(_) => break,
            }
        }

        seq = seq.wrapping_add(1);
    }
}
