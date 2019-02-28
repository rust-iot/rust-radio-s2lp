//! S2-LP Radio Driver
//! Copyright 2018 Ryan Kurte

#![no_std]
#![feature(never_type)]
extern crate embedded_hal as hal;

extern crate nb;

use hal::blocking::{spi, delay};
use hal::digital::v2::{InputPin, OutputPin};
use hal::spi::{Mode, Phase, Polarity, FullDuplex};

pub mod command;
pub mod device;
use device::{Registers, SpiCommand, XO_RCO_CONF1_PD_CLKDIV_REGMASK, XO_RCO_CONF0_REFDIV_REGMASK};

/// S2lp SPI operating mode
pub const MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// S2lp device object
pub struct S2lp<SPI, OUTPUT, INPUT, DELAY> {
    spi: SPI,
    sdn: OUTPUT,
    cs: OUTPUT,
    int: INPUT,
    delay: DELAY,
}

/// S2lp errors
pub enum Error<IoError> {
    Io(IoError),
}

impl <IoError>From<IoError> for Error<IoError> {
	fn from(e: IoError) -> Self {
		Error::Io(e)
	}
}

/// S2lp configuration
pub struct Config {
    /// Radio frequency for communication
    rf_freq_mhz: u16,
    // Clock / Crystal frequency
    clock_freq: device::ClockFreq,
}

impl<E, SPI, OUTPUT, INPUT, DELAY> S2lp<SPI, OUTPUT, INPUT, DELAY>
where
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E> + FullDuplex<u8, Error=E>,
    OUTPUT: OutputPin,
    INPUT: InputPin,
    DELAY: delay::DelayMs<u32>,
    E: Clone + core::fmt::Debug,
{
    pub fn new(spi: SPI, cs: OUTPUT, sdn: OUTPUT, int: INPUT, delay: DELAY) -> Result<Self, Error<E>> {
        let mut s2lp = S2lp { spi, sdn, cs, int, delay };

        s2lp.sdn.set_low();

        s2lp.delay.delay_ms(10);

        s2lp.sdn.set_high();


        Ok(s2lp)
    }

    /// Read data from a specified register address
    /// This consumes the provided input data array and returns a reference to this on success
    fn reg_read<'a>(&mut self, reg: Registers, data: &'a mut [u8]) -> Result<&'a [u8], Error<E>> {
        // Setup read command
        let out_buf: [u8; 2] = [SpiCommand::Read as u8, reg as u8];
        // Assert CS
        self.cs.set_low();
        // Write command
        match self.spi.write(&out_buf) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(e.into());
            }
        };
        // Transfer data
        let res = match self.spi.transfer(data) {
            Ok(r) => r,
            Err(e) => {
                self.cs.set_high();
                return Err(e.into());
            }
        };
        // Clear CS
        self.cs.set_high();
        // Return result (contains returned data)
        Ok(res)
    }

    /// Write data to a specified register address
    pub fn reg_write(&mut self, reg: Registers, data: &[u8]) -> Result<(), Error<E>> {
        // Setup write command
        let out_buf: [u8; 2] = [SpiCommand::Write as u8, reg as u8];
        // Assert CS
        self.cs.set_low();
        // Write command
        match self.spi.write(&out_buf) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(e.into());
            }
        };
        // Transfer data
        match self.spi.write(&data) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(e.into());
            }
        };
        // Clear CS
        self.cs.set_high();

        Ok(())
    }

    /// Update a register value with the provided mask
    /// new = (old & ~mask) | (val & mask)
    pub fn reg_update(&mut self, reg: Registers, mask: u8, val: u8) -> Result<(), Error<E>> {
        let mut tmp = [0u8; 1];
        let old = self.reg_read(reg.clone(), &mut tmp)?[0];
        let new = (old & !mask) | (val & mask);
        self.reg_write(reg, &[new])
    }

    /// Send a command strobe
    pub fn cmd_strobe(&mut self, cmd: device::Command) -> Result<(), Error<E>> {
        let out_buf: [u8; 2] = [SpiCommand::Strobe as u8, cmd.clone() as u8];

        // Set SMPS frequency for RX and TX commands
        match cmd {
            device::Command::Rx => {
                // sets the SMPS switching frequency to 3.12MHz.
                self.reg_write(Registers::PM_CONF3, &[0x76])?;
            },
            device::Command::Tx => {
                //sets the SMPS switching frequency to 5.46MHz about for ETSI regulation compliance.
                self.reg_write(Registers::PM_CONF3, &[0x9c])?;
            },
            _ => {}
        };

        // Execute command strobe
        self.cs.set_low();
        let res = self.spi.write(&out_buf);
        self.cs.set_high();

        res.map( |_s| () ).map_err(|e| e.into() )
    }

    /// Fetch device information (part number and version)
    pub fn device_info(&mut self) -> Result<(u8, u8), Error<E>> {
        // Read part number
        let mut data = [0u8; 1];
        let part = match self.reg_read(Registers::DEVICE_INFO1, &mut data) {
            Ok(p) => p,
            Err(e) => return Err(e),
        };

        // Read version
        let mut data = [0u8; 1];
        let version = match self.reg_read(Registers::DEVICE_INFO0, &mut data) {
            Ok(p) => p,
            Err(e) => return Err(e),
        };

        Ok((part[0], version[0]))
    }

    /// Set external clock reference mode
    pub fn set_ext_ref_mode(&mut self, mode: device::ExtRefMode) -> Result<(), Error<E>> {
        self.reg_update(Registers::XO_RCO_CONF0, device::XO_RCO_CONF0_EXT_REF_REGMASK, mode as u8)
    }

    /// Set external power supply mode
    pub fn set_ext_smps_mode(&mut self, mode: device::ExtSmpsMode) -> Result<(), Error<E>> {
        self.reg_update(Registers::PM_CONF0, device::PM_CONF0_EXT_SMPS_REGMASK, mode as u8)
    }

    /// Check whether the digital clock divider is enabled
    pub fn get_digital_divider_enabled(&mut self) -> Result<bool, Error<E>> {
        let mut tmp = [0u8; 1];
        let val = self.reg_read(Registers::XO_RCO_CONF1, &mut tmp)?[0];
        Ok(val & XO_RCO_CONF1_PD_CLKDIV_REGMASK != 0)
    }

    /// Enable the digital clock divider
    pub fn enable_digital_divider(&mut self, enabled: bool) -> Result<(), Error<E>> {
        match enabled {
            true => self.reg_update(Registers::XO_RCO_CONF1, device::XO_RCO_CONF1_PD_CLKDIV_REGMASK, 0),
            false => self.reg_update(Registers::XO_RCO_CONF1, device::XO_RCO_CONF1_PD_CLKDIV_REGMASK, device::XO_RCO_CONF1_PD_CLKDIV_REGMASK),
        }
    }

    /// Fetch reference divider enabled state
    pub fn get_reference_divider_enabled(&mut self) -> Result<bool, Error<E>> {
        let mut tmp = [0u8; 1];
        let val = self.reg_read(Registers::XO_RCO_CONF0, &mut tmp)?[0];
        Ok(val & XO_RCO_CONF0_REFDIV_REGMASK != 0)
    }

    /// Enable or disable reference divider
    pub fn enable_reference_divider(&mut self, enabled: bool) -> Result<(), Error<E>> {
        match enabled {
            true => self.reg_update(Registers::XO_RCO_CONF0, device::XO_RCO_CONF0_REFDIV_REGMASK, device::XO_RCO_CONF0_REFDIV_REGMASK),
            false => self.reg_update(Registers::XO_RCO_CONF0, device::XO_RCO_CONF0_REFDIV_REGMASK, 0),
        }
    }

    /// Set radio operating channel
    pub fn set_channel(&mut self, channel: u8) -> Result<(), Error<E>> {
        self.reg_write(Registers::CHNUM, &[channel])
    }
    
    /// Fetch radio operating channel
    pub fn get_channel(&mut self) -> Result<u8, Error<E>> {
        let mut tmp = [0u8; 1];
        self.reg_read(Registers::CHNUM, &mut tmp).map(|v| v[0])
    }

}

#[cfg(test)]
mod tests {
    extern crate embedded_hal_mock;
    use tests::embedded_hal_mock::MockError;
    use tests::embedded_hal_mock::spi::{Mock as SpiMock, Transaction as SpiTransaction};
    use tests::embedded_hal_mock::pin::{Mock as PinMock, Transaction as PinTransaction};
    use tests::embedded_hal_mock::delay::MockNoop;

    extern crate embedded_hal;
    use tests::embedded_hal::blocking::spi;
    use tests::embedded_hal::blocking::spi::{Transfer, Write};

    use super::*;

    extern crate std;
    use tests::std::boxed::Box;

    #[test]
    fn it_works() {
    
        let mut spi = SpiMock::new(&[]);
        let mut cs = PinMock::new(&[]);
        let mut sdn = PinMock::new(&[PinTransaction::set(false), PinTransaction::set(true)]);
        let mut int = PinMock::new(&[]);

        let s: Box<spi::Write<u8, Error=MockError>> = Box::new(spi.clone());

        let radio = S2lp::new(spi.clone(), cs.clone(), sdn.clone(), int.clone(), MockNoop{});

       
       sdn.done();
    }
}
