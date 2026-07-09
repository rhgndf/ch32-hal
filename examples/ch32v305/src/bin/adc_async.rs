#![no_std]
#![no_main]

use core::fmt::Write;

use ch32_hal as hal;
use ch32_hal::usb::EndpointDataBuffer512;
use ch32_hal::usbhs::{self, Driver};
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use hal::adc::{Pga, SampleTime};
use hal::{bind_interrupts, peripherals};
use heapless::String;
use panic_halt as _;

bind_interrupts!(
    struct Irqs {
        ADC => hal::adc::InterruptHandler<peripherals::ADC2>;
        USBHS => usbhs::InterruptHandler<peripherals::USBHS>;
        USBHS_WKUP => usbhs::WakeupInterruptHandler<peripherals::USBHS>;
    }
);

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI,
        ..Default::default()
    });

    let mut stream_adc = hal::adc::Adc::new(p.ADC1, Default::default());
    let mut stream_ch = p.PA5;
    let mut stream_buf = [0u16; 256];
    let mut stream = stream_adc.start_stream(
        &mut stream_ch,
        SampleTime::CYCLES239_5,
        Pga::X1,
        p.DMA1_CH1,
        &mut stream_buf,
    );

    let mut single_adc = hal::adc::Adc::new_async(p.ADC2, Default::default(), Irqs);
    let mut single_ch = p.PA6;

    let mut ep_buffer: [EndpointDataBuffer512; 4] = core::array::from_fn(|_| EndpointDataBuffer512::default());
    let driver = Driver::new(p.USBHS, Irqs, p.PB7, p.PB6, &mut ep_buffer);

    let mut config = embassy_usb::Config::new(0xC0DE, 0xCAFE);
    config.manufacturer = Some("ch32-hal");
    config.product = Some("ADC async");
    config.serial_number = Some("12345678");
    config.max_power = 100;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );

    let class = CdcAcmClass::new(&mut builder, &mut state, 512);
    let mut usb = builder.build();
    let (sender, _receiver) = class.split();
    let sender = Mutex::<NoopRawMutex, _>::new(sender);

    let usb_fut = usb.run();
    let stream_fut = async {
        loop {
            let mut sum = 0u32;
            let mut count = 0u32;
            let mut report = core::pin::pin!(Timer::after(Duration::from_millis(100)));

            loop {
                match select(
                    stream.read_half(|samples| {
                        samples.iter().fold((0u32, 0u32), |(sum, count), sample| {
                            (sum + u32::from(sample), count + 1)
                        })
                    }),
                    &mut report,
                )
                .await
                {
                    Either::First(Ok((half_sum, half_count))) => {
                        sum += half_sum;
                        count += half_count;
                    }
                    Either::First(Err(_)) => {
                        stream.clear();

                        let mut line: String<64> = String::new();
                        let _ = writeln!(line, "adc1 stream overrun");
                        let mut sender = sender.lock().await;
                        if sender.write_packet(line.as_bytes()).await.is_err() {
                            break;
                        }

                        break;
                    }
                    Either::Second(()) => {
                        let avg = if count == 0 { 0 } else { sum / count };

                        let mut line: String<64> = String::new();
                        let _ = writeln!(line, "adc1 stream avg: {} n={}", avg, count);
                        let mut sender = sender.lock().await;
                        if sender.write_packet(line.as_bytes()).await.is_err() {
                            break;
                        }

                        break;
                    }
                }
            }
        }
    };

    let single_fut = async {
        loop {
            let val = single_adc
                .convert(&mut single_ch, SampleTime::CYCLES239_5, Pga::X1)
                .await;

            let mut line: String<32> = String::new();
            let _ = writeln!(line, "adc2 single: {}", val);
            {
                let mut sender = sender.lock().await;
                let _ = sender.write_packet(line.as_bytes()).await;
            }

            Timer::after(Duration::from_millis(100)).await;
        }
    };

    join3(usb_fut, stream_fut, single_fut).await;
}
