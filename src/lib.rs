//! S2-LP Radio Driver
//! Copyright 2018 Ryan Kurte

#![no_std]
#![feature(never_type)]
#![feature(unproven)]
extern crate embedded_hal as hal;
extern crate futures;

extern crate nb;

use hal::blocking::spi;
use hal::digital::{InputPin, OutputPin};
use hal::spi::{Mode, Phase, Polarity};

pub mod command;
pub mod device;
use device::{Registers, SpiCommand};

/// eS2LP SPI operating mode
pub const MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// eS2LP device object
pub struct S2LP<SPI, OUTPUT, INPUT> {
    spi: SPI,
    sdn: OUTPUT,
    cs: OUTPUT,
    gpio: [Option<INPUT>; 4],
}

/// S2LP configuration
pub struct Config {
    /// Radio frequency for communication
    rf_freq_mhz: u16,
    // Clock / Crystal frequency
    clock_freq: device::ClockFreq,
}

impl<E, SPI, OUTPUT, INPUT> S2LP<SPI, OUTPUT, INPUT>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    OUTPUT: OutputPin,
    INPUT: InputPin,
{
    pub fn new(spi: SPI, sdn: OUTPUT, cs: OUTPUT, gpio: [Option<INPUT>; 4]) -> Result<Self, E> {
        let mut s2lp = S2LP { spi, sdn, cs, gpio };

        s2lp.sdn.set_low();

        Ok(s2lp)
    }

    /// Read data from a specified register address
    /// This consumes the provided input data array and returns a reference to this on success
    fn reg_read<'a>(&mut self, reg: u8, data: &'a mut [u8]) -> Result<&'a [u8], E> {
        // Setup read command
        let out_buf: [u8; 2] = [SpiCommand::Read as u8, reg];
        // Assert CS
        self.cs.set_low();
        // Write command
        match self.spi.write(&out_buf) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(e);
            }
        };
        // Transfer data
        let res = match self.spi.transfer(data) {
            Ok(r) => r,
            Err(e) => {
                self.cs.set_high();
                return Err(e);
            }
        };
        // Clear CS
        self.cs.set_high();
        // Return result (contains returned data)
        Ok(res)
    }

    /// Write data to a specified register address
    pub fn reg_write(&mut self, reg: u8, data: &[u8]) -> Result<(), E> {
        // Setup write command
        let out_buf: [u8; 2] = [SpiCommand::Write as u8, reg];
        // Assert CS
        self.cs.set_low();
        // Write command
        match self.spi.write(&out_buf) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(e);
            }
        };
        // Transfer data
        match self.spi.write(&data) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(e);
            }
        };
        // Clear CS
        self.cs.set_high();

        Ok(())
    }

    /// Fetch device information (part number and version)
    pub fn device_info(&mut self) -> Result<(u8, u8), E> {
        // Read part number
        let mut data = [0u8; 1];
        let part = match self.reg_read(Registers::DEVICE_INFO1 as u8, &mut data) {
            Ok(p) => p,
            Err(e) => return Err(e),
        };

        // Read version
        let mut data = [0u8; 1];
        let version = match self.reg_read(Registers::DEVICE_INFO0 as u8, &mut data) {
            Ok(p) => p,
            Err(e) => return Err(e),
        };

        Ok((part[0], version[0]))
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
