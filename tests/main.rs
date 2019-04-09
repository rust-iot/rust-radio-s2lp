
use std::env;


extern crate tokio;
use tokio::runtime::Builder;

extern crate futures;
use futures::prelude::*;

extern crate simplelog;
use simplelog::{TermLogger, LevelFilter, Config as LogConfig};

extern crate embedded_hal;

extern crate linux_embedded_hal;
use linux_embedded_hal::{Delay};

extern crate remote_hal;
use remote_hal::common::{PinMode, SpiMode};
use remote_hal::{manager::Manager, remote::Client, remote::InitRequest, remote_addr};

extern crate radio;
use radio::{Transmit, Receive, Registers};

extern crate radio_s2lp;
use radio_s2lp::{S2lp};

#[test]
#[ignore]
fn test_devices() {

    let _ = TermLogger::init(LevelFilter::Info, LogConfig::default()).unwrap();


    let spi0_name = env::var("RADIO0_SPI").expect("RADIO0_SPI environmental variable undefined");

    let spi1_name = env::var("RADIO1_SPI").expect("RADIO1_SPI environmental variable undefined");

    let cs0_name = env::var("RADIO0_CS").expect("RADIO0_CS environmental variable undefined");
    let cs1_name = env::var("RADIO1_CS").expect("RADIO1_CS environmental variable undefined");

    let reset0_name = env::var("RADIO0_RESET").expect("RADIO0_RESET environmental variable undefined");
    let reset1_name = env::var("RADIO1_RESET").expect("RADIO1_RESET environmental variable undefined");

    let int0_name = env::var("RADIO0_INT").expect("RADIO0_INT environmental variable undefined");
    let int1_name = env::var("RADIO1_INT").expect("RADIO1_INT environmental variable undefined");


    let mut rt = Builder::new()
        .blocking_threads(2)
        .core_threads(2)
        .build()
        .unwrap();


    let worker = futures::lazy(move || {
        
        Client::new(remote_addr()).map_err(|e| panic!(e) )
        .and_then(move |mut c|  {

            let reqs = vec![
                InitRequest::Spi{path: spi0_name, baud: 20_000, mode: SpiMode::Mode0},
                InitRequest::Spi{path: spi1_name, baud: 20_000, mode: SpiMode::Mode0},
                InitRequest::Pin{path: reset0_name, mode: PinMode::Output},
                InitRequest::Pin{path: reset1_name, mode: PinMode::Output},
                InitRequest::Pin{path: cs0_name, mode: PinMode::Output},
                InitRequest::Pin{path: cs1_name, mode: PinMode::Output},
                InitRequest::Pin{path: int0_name, mode: PinMode::Input},
                InitRequest::Pin{path: int1_name, mode: PinMode::Input},
            ];

            c.init_all(&reqs).map(move |devs| (c, devs) )
        })
        .and_then(move |(c, mut devs)| {

            devs.reverse();
            let spi0 = devs.pop().unwrap().spi().unwrap();
            let spi1 = devs.pop().unwrap().spi().unwrap();

            let reset0 = devs.pop().unwrap().pin().unwrap();
            let reset1 = devs.pop().unwrap().pin().unwrap();

            let cs0 = devs.pop().unwrap().pin().unwrap();
            let cs1 = devs.pop().unwrap().pin().unwrap();

            let int0 = devs.pop().unwrap().pin().unwrap();
            let int1 = devs.pop().unwrap().pin().unwrap();

            println!("Initialising radios");
        

            let mut _radio0 = S2lp::new(spi0, cs0, reset0, int0, Delay{}).expect("Failed to initialise radio0");

            //let mut _radio1 = S2lp::new(spi1, cs1, reset1, int1, Delay{}).expect("Failed to initialise radio1");


            // TODO: 

            Ok(())

        })

    });
    
    rt.block_on(worker.map(|_| () ).map_err(|e| panic!(e) )).unwrap();
}