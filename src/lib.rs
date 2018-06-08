// S2-LP Radio Driver
// Copyright 2018 Ryan Kurte

#![no_std]

#![feature(never_type)]
#![feature(unproven)]
extern crate embedded_hal as hal;
extern crate futures;

extern crate nb;

use hal::blocking::spi;
use hal::spi::{Mode, Phase, Polarity};
use hal::digital::{OutputPin};

pub mod device;
mod command;

#[doc="S2LP SPI operating mode"]
pub const MODE: Mode = Mode{ polarity: Polarity::IdleLow, phase: Phase::CaptureOnFirstTransition};

#[doc="S2LP device object"]
pub struct S2LP<SPI, SDN, IO> {
    spi: SPI,
    sdn: SDN,
    gpio: [IO; 4],
}

#[doc="S2LP configuration"]
pub struct Config {
    rf_freq_mhz: u16,
    clock_freq: device::ClockFreq,
}

impl<E, SDN, SPI, GPIO> S2LP<SPI, SDN, GPIO>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    SDN: OutputPin,
    GPIO: OutputPin,
{
    pub fn new(spi: SPI, sdn: SDN, gpio: [GPIO; 4]) -> Result<Self, E> {
        let mut s2lp = S2LP { spi, sdn, gpio };



        Ok(s2lp)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
