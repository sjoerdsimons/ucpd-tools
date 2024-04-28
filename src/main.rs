#![no_main]
#![no_std]
use byteorder::{ByteOrder, LittleEndian};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts, gpio, peripherals,
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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
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

    let mut led_spy = gpio::Output::new(p.PC12, gpio::Level::Low, gpio::Speed::Low);

    let mut ucpd1 = ucpd::Ucpd::new(p.UCPD1, Irqs, p.PA8, p.PB15);
    let (_cc_phy, mut pd_phy) = ucpd1.split_pd_phy(p.DMA1_CH1, p.DMA1_CH2, CcSel::CC1);

    // vstate not monitored by phy due to external pulldown
    /*
    let vstate = cc_phy.vstate();
    info!("vstate cc1: {} cc2: {}", vstate.0 as u8, vstate.1 as u8);
    let vstate = cc_phy.wait_for_vstate_change().await;
    info!(
        "vstate change cc1: {} cc2: {}",
        vstate.0 as u8, vstate.1 as u8
    );
    */

    uart.write(b"started\r\n").await;
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
        led_spy.toggle();
        //uart.write(&data[..r]).await.unwrap();
        //Timer::after_millis(500).await;
    }
}
