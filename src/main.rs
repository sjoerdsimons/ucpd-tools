#![no_main]
#![no_std]
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_stm32::init(Default::default());

    info!("STARTED");
    panic!("PANIEK");
}
