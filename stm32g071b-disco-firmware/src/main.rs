#![no_main]
#![no_std]
use core::fmt::Display;

use analyzer_cbor::AnalyserData;
use byteorder::{ByteOrder, LittleEndian};
use defmt::{info, trace};
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_futures::select::{select, select_array, Either};
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio,
    i2c::{self, I2c},
    mode::Async,
    peripherals::{self, PA3, PA6, PA7, PC10, PC11, USART3},
    spi::Spi,
    time::Hertz,
    ucpd::{self, CcSel},
    usart::{self, BufferedUart},
};
use embassy_sync::{
    blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex},
    channel::Channel,
    mutex::Mutex,
};
use embassy_time::Timer;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    mono_font::{ascii::FONT_6X12, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
    Drawable,
};
use embedded_hal_bus::spi::NoDelay;
use embedded_io_async::Write;
use minicbor::encode::write::Cursor;
use ssd1306::{prelude::*, Ssd1306};
use usb_pd::message::Message;

use panic_probe as _;

mod watch;
use watch::{Watch, WatchPublisher, WatchSubscriber};

// Channel for PD messages
static PD_MESSAGES: Channel<ThreadModeRawMutex, PdData, 8> = Channel::new();
// Watch for voltage sensors
static SENSOR_DATA: Watch<ThreadModeRawMutex, SensorData, 2> = Watch::new();

#[derive(Clone)]
struct SensorData {
    vbus_v: f64,
    vbus_a: f64,
    cc1_v: f64,
    cc2_v: f64,
}

struct Precision2(f64);
impl Display for Precision2 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let u = ((self.0 * 100.0) as i64).abs();
        f.write_fmt(format_args!("{}.{:02}", u / 100, u % 100))
    }
}

bind_interrupts!(struct Irqs {
    USART3_4_LPUART1 => usart::BufferedInterruptHandler<peripherals::USART3>;
    UCPD1_2 => ucpd::InterruptHandler<peripherals::UCPD1>;
    I2C1 => i2c::EventInterruptHandler<peripherals::I2C1>,
            i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[derive(Copy, Clone)]
struct PdData {
    data: [u8; 128],
    len: usize,
}

async fn send_data(uart: &mut BufferedUart<'_, USART3>, data: &AnalyserData<'_>) {
    let mut cursor = Cursor::new([0u8; 128]);
    minicbor::encode(data, &mut cursor).unwrap();
    let len = cursor.position();
    uart.write_all(&cursor.get_ref()[..len]).await.unwrap();
}

// needs spi (spi1, sck: pa1, mosi: pa2), dc pin: pa7, reset: pa6, cs: pa3
#[embassy_executor::task]
async fn display(
    spi: Spi<'static, peripherals::SPI1, Async>,
    dc: PA7,
    reset: PA6,
    cs: PA3,
    mut sensors: WatchSubscriber<'static, SensorData>,
) {
    let dc = gpio::Output::new(dc, gpio::Level::Low, gpio::Speed::Low);
    let cs = gpio::Output::new(cs, gpio::Level::Low, gpio::Speed::Low);
    let mut reset = gpio::Output::new(reset, gpio::Level::Low, gpio::Speed::Low);

    let device = embedded_hal_bus::spi::ExclusiveDevice::new(spi, cs, NoDelay).unwrap();

    let interface = SPIInterface::new(device, dc);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display
        .reset(&mut reset, &mut embassy_time::Delay {})
        .unwrap();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X12)
        .text_color(BinaryColor::On)
        .build();

    loop {
        use core::fmt::Write;
        let mut info = heapless::String::<64>::new();
        display.clear(BinaryColor::Off).unwrap();

        if let Some(data) = sensors.last() {
            let _ = write!(
                info,
                "VBUS: {}V {}A\nCC1: {}V\nCC2: {}V",
                Precision2(data.vbus_v),
                Precision2(data.vbus_a),
                Precision2(data.cc1_v),
                Precision2(data.cc2_v),
            );
        } else {
            let _ = write!(info, "*****",);
        }

        // Vbus
        Text::with_baseline(&info, Point::new(4, 8), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();
        Timer::after_millis(250).await;
    }
}

#[embassy_executor::task]
async fn serial_output(
    uart: USART3,
    tx: PC11,
    rx: PC10,
    mut sensors: WatchSubscriber<'static, SensorData>,
) {
    let mut config = embassy_stm32::usart::Config::default();
    config.baudrate = 115200;
    let mut tx_buf = [0u8; 256];
    let mut rx_buf = [0u8; 256];
    let mut uart = BufferedUart::new(uart, Irqs, tx, rx, &mut tx_buf, &mut rx_buf, config).unwrap();

    loop {
        let next = select(PD_MESSAGES.receive(), sensors.wait_next()).await;
        match next {
            Either::First(pd) => {
                send_data(
                    &mut uart,
                    &AnalyserData::PdSpy {
                        data: pd.data.as_slice()[..pd.len].into(),
                    },
                )
                .await
            }
            Either::Second(data) => {
                send_data(
                    &mut uart,
                    &AnalyserData::Power {
                        label: "vbus",
                        voltage: data.vbus_v,
                        current: Some(data.vbus_a),
                    },
                )
                .await;
                send_data(
                    &mut uart,
                    &AnalyserData::Power {
                        label: "cc1",
                        voltage: data.cc1_v,
                        current: None,
                    },
                )
                .await;
                send_data(
                    &mut uart,
                    &AnalyserData::Power {
                        label: "cc2",
                        voltage: data.cc2_v,
                        current: None,
                    },
                )
                .await;
            }
        }
    }
}

#[embassy_executor::task]
async fn sensor_monitor(
    i2c: I2c<'static, peripherals::I2C1, Async>,
    publisher: WatchPublisher<'static, SensorData>,
) {
    let i2c: Mutex<NoopRawMutex, _> = Mutex::new(i2c);
    let mut vbus = ina226::INA226::new(I2cDevice::new(&i2c), 0x40);
    vbus.callibrate(0.015, 5.0)
        .await
        .expect("Failed to calibrate vbus");
    let mut cc1 = ina226::INA226::new(I2cDevice::new(&i2c), 0x41);
    cc1.callibrate(0.015, 5.0)
        .await
        .expect("Failed to calibrate cc1");
    let mut cc2 = ina226::INA226::new(I2cDevice::new(&i2c), 0x42);
    cc2.callibrate(0.015, 5.0)
        .await
        .expect("Failed to calibrate cc2");

    loop {
        let vbus_v = vbus.bus_voltage_millivolts().await.unwrap_or_default() / 1000.0;
        let vbus_a = vbus
            .current_amps()
            .await
            .unwrap_or_default()
            .unwrap_or_default();

        let cc1_v = cc1.bus_voltage_millivolts().await.unwrap_or_default() / 1000.0;
        let cc1_a = cc1
            .current_amps()
            .await
            .unwrap_or_default()
            .unwrap_or_default();

        let cc2_v = cc2.bus_voltage_millivolts().await.unwrap_or_default() / 1000.0;
        let cc2_a = cc2
            .current_amps()
            .await
            .unwrap_or_default()
            .unwrap_or_default();
        trace!("VBus: {}V {}A", vbus_v, vbus_a);
        trace!("CC1: {}V {}A", cc1_v, cc1_a);
        trace!("CC2: {}V {}A", cc2_v, cc2_a);
        publisher.publish(SensorData {
            vbus_v,
            vbus_a,
            cc1_v,
            cc2_v,
        });
        Timer::after_millis(50).await;
    }
}

#[embassy_executor::task]
async fn power_monitor(
    mut stlk_on: ExtiInput<'static>,
    smps_on: ExtiInput<'static>,
    mut smps_dis: gpio::Output<'static>,
) {
    loop {
        info!("SMPS: {}", smps_on.is_high());

        // Assume we're powered from vbus (smps). If the stlk comes on turn off smps
        stlk_on.wait_for_high().await;
        info!("Powered from ST Link");
        smps_dis.set_high();

        // If the ST link gets removed re-enable the smps, though likely the system will be reset
        // before
        stlk_on.wait_for_low().await;
        info!("ST Link removed");
        smps_dis.set_low();
    }
}

#[embassy_executor::task]
async fn joystick_monitor(
    mut joy_sel: ExtiInput<'static>,
    mut joy_left: ExtiInput<'static>,
    mut joy_down: ExtiInput<'static>,
    mut joy_right: ExtiInput<'static>,
    mut joy_up: ExtiInput<'static>,
) {
    loop {
        // TODO this restart the watch every time, which may mean events could be missed?
        match select_array([
            joy_sel.wait_for_any_edge(),
            joy_left.wait_for_any_edge(),
            joy_down.wait_for_any_edge(),
            joy_right.wait_for_any_edge(),
            joy_up.wait_for_any_edge(),
        ])
        .await
        {
            ((), 0) => info!("Joystick select: {}", joy_sel.is_high()),
            ((), 1) => info!("Joystick left: {}", joy_left.is_high()),
            ((), 2) => info!("Joystick down: {}", joy_down.is_high()),
            ((), 3) => info!("Joystick right: {}", joy_right.is_high()),
            ((), 4) => info!("Joystick up: {}", joy_up.is_high()),
            _ => unreachable!(""),
        }
    }
}

#[embassy_executor::task]
async fn door_monitor(
    mut door_sense: ExtiInput<'static>,
    mut en_cc1: gpio::Output<'static>,
    mut en_cc2: gpio::Output<'static>,
    mut en_rd: gpio::Output<'static>,
    mut led_spy: gpio::Output<'static>,
) {
    loop {
        info!("Door: {}", door_sense.is_high());
        if door_sense.is_high() {
            led_spy.set_high();
            /* spy mode; disable RD */
            en_rd.set_low();
            // Pass through both cc lines
            en_cc1.set_high();
            en_cc2.set_high();
        } else {
            led_spy.set_low();
            // standalone, so enable rd resistor
            en_rd.set_high();
            // rd is on cc1 so only pass that though
            en_cc1.set_high();
            en_cc2.set_low();
        }
        door_sense.wait_for_any_edge().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    info!("Started");

    // Setup door monitor
    let door_sense = ExtiInput::new(p.PC8, p.EXTI8, gpio::Pull::Up);
    let en_cc1 = gpio::Output::new(p.PB10, gpio::Level::High, gpio::Speed::Low);
    let en_cc2 = gpio::Output::new(p.PB11, gpio::Level::High, gpio::Speed::Low);
    let en_rb = gpio::Output::new(p.PB12, gpio::Level::High, gpio::Speed::Low);
    let led_spy = gpio::Output::new(p.PC12, gpio::Level::Low, gpio::Speed::Low);

    spawner
        .spawn(door_monitor(door_sense, en_cc1, en_cc2, en_rb, led_spy))
        .unwrap();

    // Setup joystick monitor
    spawner
        .spawn(joystick_monitor(
            ExtiInput::new(p.PC0, p.EXTI0, gpio::Pull::Down),
            ExtiInput::new(p.PC1, p.EXTI1, gpio::Pull::Down),
            ExtiInput::new(p.PC2, p.EXTI2, gpio::Pull::Down),
            ExtiInput::new(p.PC3, p.EXTI3, gpio::Pull::Down),
            ExtiInput::new(p.PC4, p.EXTI4, gpio::Pull::Down),
        ))
        .unwrap();

    // Setup power monitor
    spawner
        .spawn(power_monitor(
            ExtiInput::new(p.PA11, p.EXTI11, gpio::Pull::None),
            ExtiInput::new(p.PA12, p.EXTI12, gpio::Pull::None),
            gpio::Output::new(p.PA0, gpio::Level::Low, gpio::Speed::Low),
        ))
        .unwrap();

    // Setup sensor monitor
    spawner
        .spawn(sensor_monitor(
            I2c::new(
                p.I2C1,
                p.PB6,
                p.PB7,
                Irqs,
                p.DMA1_CH3,
                p.DMA1_CH4,
                Hertz(100_000),
                Default::default(),
            ),
            SENSOR_DATA.publish(),
        ))
        .unwrap();

    // Serial output
    spawner
        .spawn(serial_output(
            p.USART3,
            p.PC11,
            p.PC10,
            SENSOR_DATA.subscribe().unwrap(),
        ))
        .unwrap();

    // Display output
    //
    let config = embassy_stm32::spi::Config::default();
    let spi = embassy_stm32::spi::Spi::new_txonly(p.SPI1, p.PA1, p.PA2, p.DMA1_CH5, config);

    spawner
        .spawn(display(
            spi,
            p.PA7,
            p.PA6,
            p.PA3,
            SENSOR_DATA.subscribe().unwrap(),
        ))
        .unwrap();

    // Monitor CC; TODO swithc dynamically to CC2 if needed
    // _cc_phy is unused as there is an external Rd in both modes
    let mut led_cc = gpio::Output::new(p.PD5, gpio::Level::Low, gpio::Speed::Low);
    let (_cc_phy, mut pd_phy) = ucpd::Ucpd::new(p.UCPD1, Irqs, p.PA8, p.PB15).split_pd_phy(
        p.DMA1_CH1,
        p.DMA1_CH2,
        CcSel::CC1,
    );

    loop {
        let mut buf = [0u8; 128];
        match pd_phy.receive(&mut buf).await {
            Ok(n) => {
                info!("USB PD RX: {=[u8]:?}", &buf[..n]);
                let header = usb_pd::header::Header(LittleEndian::read_u16(&buf));
                info!(
                    "USB PD RX Header ({:?}): {:?}",
                    header.message_type(),
                    header,
                );
                let message = Message::parse(header, &buf[2..]);
                info!("Message: {:?}", &message);

                let d = PdData { data: buf, len: n };
                PD_MESSAGES.send(d).await;
            }
            Err(e) => info!("USB PD RX: {}", e),
        }
        led_cc.toggle();
    }
}
