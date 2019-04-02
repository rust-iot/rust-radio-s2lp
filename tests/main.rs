
use std::env;


extern crate tokio;
use tokio::runtime::Runtime;

extern crate futures;
use futures::prelude::*;

extern crate embedded_hal;

extern crate linux_embedded_hal;
use linux_embedded_hal::{Delay};

extern crate remote_hal;
use remote_hal::common::{PinMode, SpiMode};
use remote_hal::{manager::Manager, remote::Client, remote_addr};

extern crate radio;
use radio::{Transmit, Receive, Registers};

extern crate radio_s2lp;
use radio_s2lp::{S2lp};

#[test]
#[ignore]
fn test_devices() {
    let spi0_name = env::var("RADIO0_SPI").expect("RADIO0_SPI environmental variable undefined");

    let spi1_name = env::var("RADIO1_SPI").expect("RADIO1_SPI environmental variable undefined");

    let cs0_name = env::var("RADIO0_CS").expect("RADIO0_CS environmental variable undefined");
    let cs1_name = env::var("RADIO1_CS").expect("RADIO1_CS environmental variable undefined");

    let reset0_name = env::var("RADIO0_RESET").expect("RADIO0_RESET environmental variable undefined");
    let reset1_name = env::var("RADIO1_RESET").expect("RADIO1_RESET environmental variable undefined");

    let int0_name = env::var("RADIO0_INT").expect("RADIO0_INT environmental variable undefined");
    let int1_name = env::var("RADIO1_INT").expect("RADIO1_INT environmental variable undefined");


    let mut rt = Runtime::new().unwrap();


    let connect = Client::new(remote_addr()).map_err(|e| panic!(e) );

    rt.block_on(connect.and_then(move |mut c| -> Result<(), ()> {

        let spi0 = c.spi(&spi0_name, 20_000, SpiMode::Mode0).wait().expect("Failed to open SPI0");
        let spi1 = c.spi(&spi1_name, 20_000, SpiMode::Mode0).wait().expect("Failed to open SPI1");

        let cs0 = c.pin(&cs0_name, PinMode::Output).wait().expect("Failed to open CS0");
        let cs1 = c.pin(&cs1_name, PinMode::Output).wait().expect("Failed to open CS1");

        let reset0 = c.pin(&reset0_name, PinMode::Output).wait().expect("Failed to open RESET0");
        let reset1 = c.pin(&reset1_name, PinMode::Output).wait().expect("Failed to open RESET1");

        let int0 = c.pin(&int0_name, PinMode::Input).wait().expect("Failed to open SLEEP0");
        let int1 = c.pin(&int1_name, PinMode::Input).wait().expect("Failed to open SLEEP1");


        println!("Initialising radios");
    

        let mut _radio0 = S2lp::new(spi0, cs0, reset0, int0, Delay{}).expect("Failed to initialise radio0");

        let mut _radio1 = S2lp::new(spi1, cs1, reset1, int1, Delay{}).expect("Failed to initialise radio1");


        // TODO: 

        Ok(())

    })).unwrap();
}