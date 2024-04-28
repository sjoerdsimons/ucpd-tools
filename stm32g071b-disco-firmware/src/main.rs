#![no_main]
#![no_std]
use analyzer_cbor::AnalyserData;
use byteorder::{ByteOrder, LittleEndian};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::select_array;
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio, peripherals,
    ucpd::{self, CcSel},
    usart::{self, BufferedUart},
};
use embedded_io_async::Write;
use minicbor::encode::write::Cursor;
use usb_pd::message::Message;

use panic_probe as _;

bind_interrupts!(struct Irqs {
    USART3_4_LPUART1 => usart::BufferedInterruptHandler<peripherals::USART3>;
    UCPD1_2 => ucpd::InterruptHandler<peripherals::UCPD1>;
});

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

    info!("STARTED");
    let mut config = embassy_stm32::usart::Config::default();
    config.baudrate = 115200;

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

    // Monitor CC; TODO swithc dynamically to CC2 if needed
    // _cc_phy is unused as there is an external Rd in both modes
    let mut led_cc = gpio::Output::new(p.PD5, gpio::Level::Low, gpio::Speed::Low);
    let (_cc_phy, mut pd_phy) = ucpd::Ucpd::new(p.UCPD1, Irqs, p.PA8, p.PB15).split_pd_phy(
        p.DMA1_CH1,
        p.DMA1_CH2,
        CcSel::CC1,
    );

    let mut tx_buf = [0u8; 256];
    let mut rx_buf = [0u8; 256];
    let mut uart = BufferedUart::new(
        p.USART3,
        Irqs,
        p.PC11,
        p.PC10,
        &mut tx_buf,
        &mut rx_buf,
        config,
    )
    .unwrap();

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

                let tosend = AnalyserData::PdSpy {
                    data: buf[..n].into(),
                };

                //let mut cbor = [0u8; 128];
                let mut cursor = Cursor::new([0u8; 128]);
                minicbor::encode(&tosend, &mut cursor).unwrap();
                let len = cursor.position();
                uart.write_all(&cursor.get_ref()[..len]).await.unwrap();
            }
            Err(e) => info!("USB PD RX: {}", e),
        }
        led_cc.toggle();
    }
}
