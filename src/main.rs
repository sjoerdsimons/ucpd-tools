#![no_main]
#![no_std]
use byteorder::{ByteOrder, LittleEndian};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{self, Pin},
    peripherals,
    ucpd::{self, CcSel},
    usart::{self, BufferedUart, Uart},
};
//use embassy_time::Timer;
use embedded_io_async::{Read, Write};
use panic_probe as _;
use usb_pd::message::Message;

bind_interrupts!(struct Irqs {
    USART3_4_LPUART1 => usart::BufferedInterruptHandler<peripherals::USART3>;
    UCPD1_2 => ucpd::InterruptHandler<peripherals::UCPD1>;
});

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
    let config = embassy_stm32::usart::Config::default();

    let mut tx_buf = [0u8; 128];
    let mut rx_buf = [0u8; 128];
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

    // Setup door monitor
    let door_sense = embassy_stm32::exti::ExtiInput::new(p.PC8, p.EXTI8, gpio::Pull::Up);
    let en_cc1 = gpio::Output::new(p.PB10, gpio::Level::High, gpio::Speed::Low);
    let en_cc2 = gpio::Output::new(p.PB11, gpio::Level::High, gpio::Speed::Low);
    let en_rb = gpio::Output::new(p.PB12, gpio::Level::High, gpio::Speed::Low);
    let led_spy = gpio::Output::new(p.PC12, gpio::Level::Low, gpio::Speed::Low);

    spawner
        .spawn(door_monitor(door_sense, en_cc1, en_cc2, en_rb, led_spy))
        .unwrap();

    uart.write(b"started\r\n").await;
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
            }
            Err(e) => info!("USB PD RX: {}", e),
        }
        led_cc.toggle();
        //uart.write(&data[..r]).await.unwrap();
        //Timer::after_millis(500).await;
    }
}
