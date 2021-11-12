//! S2-LP Device definitions
//! 
//! Copyright 2018 Ryan Kurte

use core::convert::Infallible;

use modular_bitfield::prelude::*;
use num_enum::{IntoPrimitive, TryFromPrimitive, TryFromPrimitiveError};

/// SPI command modes
#[derive(Copy, Clone, Debug)]
pub enum SpiCommand {
    Write = 0x00,  // SPI Write register command
    Read = 0x01,   // SPI Read register command
    Strobe = 0x80, // SPI Strobe command
}

pub const XO_RCO_CONF0_REFDIV_REGMASK: u8 = 0x08;
pub const XO_RCO_CONF1_PD_CLKDIV_REGMASK: u8 = 0x10;

// TODO: how to represent different enums for Gpio Out and In modes?

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio0Conf {
    pub gpio_select: GpioOutSelect,
    #[skip]
    __: B1,
    pub gpio_mode: GpioMode,
}

impl radio::Register for Gpio0Conf {
    const ADDRESS: u8 = Registers::GPIO0_CONF as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio1Conf {
    pub gpio_select: GpioOutSelect,
    #[skip]
    __: B1,
    pub gpio_mode: GpioMode,
}

impl radio::Register for Gpio1Conf {
    const ADDRESS: u8 = Registers::GPIO1_CONF as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio2Conf {
    pub gpio_select: GpioOutSelect,
    #[skip]
    __: B1,
    pub gpio_mode: GpioMode,
}

impl radio::Register for Gpio2Conf {
    const ADDRESS: u8 = Registers::GPIO2_CONF as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Gpio3Conf {
    pub gpio_select: GpioOutSelect,
    #[skip]
    __: B1,
    pub gpio_mode: GpioMode,
}

impl radio::Register for Gpio3Conf {
    const ADDRESS: u8 = Registers::GPIO3_CONF as u8;
    type Word = u8;
    type Error = Infallible;
}


/// GPIO Pin Modes
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum GpioMode {
    DigitalInput = 0b01,
    DigitalOutputLowPower = 0b00,
    DigitalOutputHighPower = 0b11,
}

/// GPIO Pin Operation
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 5]
pub enum GpioOutSelect {
    NIrq = 0,   // nIRQ (interrupt request, active low)
    Por = 1,    // POR inverted (active low)
    Wake = 2,   // Wake-up timer expiration: ‘1’ when WUT has expired
    BattLow = 3,    // Low battery detection: ‘1’ when battery is below threshold setting
    TxDataClock = 4,     //TX data internal clock output (TX data are sampled on the rising edge of it)
    TxState = 5,     //TX state outputs a command information coming from the RADIO_TX block
    FifoAlmostEmpty = 6,     //TX/RX FIFO almost empty flag
    FifoAlmostFull = 7,     //TX/RX FIFO almost full flag
    RxData = 8,     //RX data output
    RxClock = 9,     //RX clock output (recovered from received data)
    RxState = 10,       // RX state indication: ‘1’ when the S2-LP is transiting in the RX state
    DeviceAwake = 11,   // Device in a state other than SLEEP or STANDBY: ‘0’ when in SLEEP/STANDBY
    DeviceStandby = 12,   // Device in STANDBY state
    AntennaSw = 13,  // Antenna switch signal used for antenna diversity
    PreambleValid = 14,    // Valid preamble detected flag
    SyncValid = 15, // Sync word detected flag
    RssiAboveThreshold = 16, // RSSI above threshold (same indication of CS register)
    Reserved = 17, // Reserved
    TxRxMode = 18,   // TX or RX mode indicator (to enable an external range extender)
    Vdd = 19,  // VDD (to emulate an additional GPIO of the MCU, programmable by SPI)
    Gnd = 20,  // GND (to emulate an additional GPIO of the MCU, programmable by SPI)
    ExternalSmpsEn = 21, // External SMPS enable signal (active high)
    DeviceSleep = 22,   // Device in SLEEP state
    DeviceReady = 23,   // Device in READY state
    DeviceLock = 24,   // Device in LOCK state
    DeviceWaitLock = 25,   // Device waiting for a high level of the lock-detector output signal
    TxDataOok = 26,  // TX_DATA_OOK signal (internal control signal generated in the OOK analog smooth mode)
    DeviceWaitReady2 = 27,   // Device waiting for a high level of the READY2 signal from XO
    DeviceWaitTimer = 28,   // Device waiting for timer expiration to allow PM block settling
    DeviceWaitVco = 29,   // Device waiting for end of VCO calibration
    DeviceSynthEn = 30,   // Device enables the full circuitry of the SYNTH block
    Reserved2 = 31, // Reserved
}

#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 5]
pub enum GpioInSelect {
    TxCommand = 0,
    RxCommand = 1,
    TxData = 2,
    Wake = 3,
    ExternalClock = 4,
}


#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt3 {
    /// Set the charge pump current according to the XTAL frequency
    pub pll_cp_isel: B3,
    /// Synthesizer band select. This parameter selects the out-of loop divide factor of the synthesizer
    pub band_sel: B1,
    /// MSB bits of the PLL programmable divider
    pub synt_27_24: B4,
}

impl radio::Register for Synt3 {
    const ADDRESS: u8 = Registers::SYNT3 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt2 {
    /// Intermediate bits of the PLL programmable divider
    pub synt_23_16: u8,
}

impl radio::Register for Synt2 {
    const ADDRESS: u8 = Registers::SYNT2 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt1 {
    /// Intermediate bits of the PLL programmable divider
    pub synt_15_8: u8,
}

impl radio::Register for Synt1 {
    const ADDRESS: u8 = Registers::SYNT1 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Synt0 {
    /// Lower bits of the PLL programmable divider
    pub synt_7_0: u8,
}

impl radio::Register for Synt0 {
    const ADDRESS: u8 = Registers::SYNT0 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IfOffsetAna {
    /// Intermediate frequency setting for the analog RF synthesizer
    pub if_ana: u8,
}

impl radio::Register for IfOffsetAna {
    const ADDRESS: u8 = Registers::IF_OFFSET_ANA as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IfOffsetDig {
    /// Intermediate frequency setting for the digital shift-to-baseband circuits
    pub if_dig: u8,
}

impl radio::Register for IfOffsetDig {
    const ADDRESS: u8 = Registers::IF_OFFSET_DIG as u8;
    type Word = u8;
    type Error = Infallible;
}


#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct ChSpace {
    /// Channel spacing setting
    pub ch_space: u8,
}

impl radio::Register for ChSpace {
    const ADDRESS: u8 = Registers::CH_SPACE as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct ChNum {
    /// Channel number. This value is multiplied by the channel spacing and 
    /// added to the synthesizer base frequency to generate the actual RF carrier frequency
    pub ch_num: u8,
}

impl radio::Register for ChNum {
    const ADDRESS: u8 = Registers::CH_NUM as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod4 {
    /// The MSB of the mantissa value of the data rate equation
    pub datarate_m_15_8: u8,
}

impl radio::Register for Mod4 {
    const ADDRESS: u8 = Registers::MOD4 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod3 {
    /// The LSB of the mantissa value of the data rate equation
    pub datarate_m_7_0: u8,
}

impl radio::Register for Mod3 {
    const ADDRESS: u8 = Registers::MOD3 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod2 {
    /// Modulation type
    pub mod_type: ModType,
    /// Datarate exponent
    pub datarate_e: B4
}

impl radio::Register for Mod2 {
    const ADDRESS: u8 = Registers::MOD2 as u8;
    type Word = u8;
    type Error = Infallible;
}


/// Modulation modes
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 4]
pub enum ModType {
    /// 2-FSK
    Mod2Fsk = 0,
    /// 4-FSK
    Mod4Fsk = 1,
    /// 2-GFSK BT=1.0
    Mod2Gfsk = 2,
    /// 4-GFSK BT=1.0
    Mod4Gfsk = 3,
    /// ASK/OOK
    ModAskOok = 5,
    /// 2-GFSK BT=0.5
    Mod2GfskBt05 = 10,
    /// 4-GFSK BT=0.5
    Mod4GfskBt05 = 11,
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod1 {
    /// enable the PA power interpolator
    pub pa_interp_en: bool,
    /// enable frequency interpolator for the GFSK shaping
    pub mod_interp_en: bool,
    /// Select the constellation map for 4-(G)FSK or 2-(G)FSK modulations
    pub const_map: B2,
    /// The exponent value of the frequency deviation equation
    pub fdev_e: B4,
}

impl radio::Register for Mod1 {
    const ADDRESS: u8 = Registers::MOD1 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Mod0 {
    /// The mantissa value of the frequency deviation equation
    pub fdev_m: u8,
}

impl radio::Register for Mod0 {
    const ADDRESS: u8 = Registers::MOD0 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct ChFlt {
    /// The mantissa value of the receiver channel filter
    pub chflt_m: B4,
    /// The exponent value of the receiver channel filter
    pub chflt_e: B4,
}

impl radio::Register for ChFlt {
    const ADDRESS: u8 = Registers::CHFLT as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Afc2 {
    pub raw: u8,
}

impl radio::Register for Afc2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Afc1 {
    pub raw: u8,
}

impl radio::Register for Afc1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Afc0 {
    pub raw: u8,
}

impl radio::Register for Afc0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiFlt {
    pub raw: u8,
}

impl radio::Register for RssiFlt {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiTh {
    pub raw: u8,
}

impl radio::Register for RssiTh {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl4 {
    pub raw: u8,
}

impl radio::Register for Agcctrl4 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl3 {
    pub raw: u8,
}

impl radio::Register for Agcctrl3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl2 {
    pub raw: u8,
}

impl radio::Register for Agcctrl2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl1 {
    pub raw: u8,
}

impl radio::Register for Agcctrl1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Agcctrl0 {
    pub raw: u8,
}

impl radio::Register for Agcctrl0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct AntSelectConf {
    pub raw: u8,
}

impl radio::Register for AntSelectConf {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Clockrec2 {
    pub raw: u8,
}

impl radio::Register for Clockrec2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Clockrec1 {
    pub raw: u8,
}

impl radio::Register for Clockrec1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl6 {
    pub raw: u8,
}

impl radio::Register for Pcktctrl6 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl5 {
    pub raw: u8,
}

impl radio::Register for Pcktctrl5 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl4 {
    pub raw: u8,
}

impl radio::Register for Pcktctrl4 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl3 {
    pub raw: u8,
}

impl radio::Register for Pcktctrl3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FixVarLen {
    pub raw: u8,
}

impl radio::Register for FixVarLen {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktctrl1 {
    pub raw: u8,
}

impl radio::Register for Pcktctrl1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktlen1 {
    pub raw: u8,
}

impl radio::Register for Pcktlen1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Pcktlen0 {
    pub raw: u8,
}

impl radio::Register for Pcktlen0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync3 {
    pub raw: u8,
}

impl radio::Register for Sync3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync2 {
    pub raw: u8,
}

impl radio::Register for Sync2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync1 {
    pub raw: u8,
}

impl radio::Register for Sync1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Sync0 {
    pub raw: u8,
}

impl radio::Register for Sync0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Qi {
    pub raw: u8,
}

impl radio::Register for Qi {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktPstmbl {
    pub raw: u8,
}

impl radio::Register for PcktPstmbl {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct LdcTimerMult {
    pub raw: u8,
}

impl radio::Register for LdcTimerMult {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct AutoPcktFlt {
    pub raw: u8,
}

impl radio::Register for AutoPcktFlt {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Protocol0 {
    pub raw: u8,
}

impl radio::Register for Protocol0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig3 {
    pub raw: u8,
}

impl radio::Register for FifoConfig3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig2 {
    pub raw: u8,
}

impl radio::Register for FifoConfig2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig1 {
    pub raw: u8,
}

impl radio::Register for FifoConfig1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FifoConfig0 {
    pub raw: u8,
}

impl radio::Register for FifoConfig0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct DestVsSourceAddr {
    pub raw: u8,
}

impl radio::Register for DestVsSourceAddr {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals4 {
    pub raw: u8,
}

impl radio::Register for PcktFltGoals4 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals3 {
    pub raw: u8,
}

impl radio::Register for PcktFltGoals3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals2 {
    pub raw: u8,
}

impl radio::Register for PcktFltGoals2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals1 {
    pub raw: u8,
}

impl radio::Register for PcktFltGoals1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PcktFltGoals0 {
    pub raw: u8,
}

impl radio::Register for PcktFltGoals0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers5 {
    pub raw: u8,
}

impl radio::Register for Timers5 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers4 {
    pub raw: u8,
}

impl radio::Register for Timers4 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers3 {
    pub raw: u8,
}

impl radio::Register for Timers3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers2 {
    pub raw: u8,
}

impl radio::Register for Timers2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers1 {
    pub raw: u8,
}

impl radio::Register for Timers1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Timers0 {
    pub raw: u8,
}

impl radio::Register for Timers0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf3 {
    pub raw: u8,
}

impl radio::Register for CsmaConf3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf2 {
    pub raw: u8,
}

impl radio::Register for CsmaConf2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf1 {
    pub raw: u8,
}

impl radio::Register for CsmaConf1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CsmaConf0 {
    pub raw: u8,
}

impl radio::Register for CsmaConf0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask3 {
    pub raw: u8,
}

impl radio::Register for IrqMask3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask2 {
    pub raw: u8,
}

impl radio::Register for IrqMask2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask1 {
    pub raw: u8,
}

impl radio::Register for IrqMask1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqMask0 {
    pub raw: u8,
}

impl radio::Register for IrqMask0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct FastRxTimer {
    pub raw: u8,
}

impl radio::Register for FastRxTimer {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower8 {
    pub raw: u8,
}

impl radio::Register for PaPower8 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower7 {
    pub raw: u8,
}

impl radio::Register for PaPower7 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower6 {
    pub raw: u8,
}

impl radio::Register for PaPower6 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower5 {
    pub raw: u8,
}

impl radio::Register for PaPower5 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower4 {
    pub raw: u8,
}

impl radio::Register for PaPower4 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower3 {
    pub raw: u8,
}

impl radio::Register for PaPower3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower2 {
    pub raw: u8,
}

impl radio::Register for PaPower2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower1 {
    pub raw: u8,
}

impl radio::Register for PaPower1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaPower0 {
    pub raw: u8,
}

impl radio::Register for PaPower0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaConfig1 {
    pub raw: u8,
}

impl radio::Register for PaConfig1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PaConfig0 {
    pub raw: u8,
}

impl radio::Register for PaConfig0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct SynthConfig2 {
    pub raw: u8,
}

impl radio::Register for SynthConfig2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoConfig {
    pub raw: u8,
}

impl radio::Register for VcoConfig {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrIn2 {
    pub raw: u8,
}

impl radio::Register for VcoCalibrIn2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrIn1 {
    pub raw: u8,
}

impl radio::Register for VcoCalibrIn1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrIn0 {
    pub raw: u8,
}

impl radio::Register for VcoCalibrIn0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct XoRcoConf1 {
    #[skip]
    __: B3,
    /// disable both dividers of digital clock
    pub pd_clkdiv: bool,
    #[skip]
    __: B4,
}

impl radio::Register for XoRcoConf1 {
    const ADDRESS: u8 = Registers::XO_RCO_CONF1 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct XoRcoConf0 {
    /// Set external reference mode
    pub ext_ref: ExtRefMode,
    /// Set the driver gm of the XO at start up
    pub gm_conf: B3,
    /// Enable the the reference clock divider
    pub ref_div: bool,
    #[skip]
    __: B1,
    /// Enable external RCO, the 34.7 kHz signal must be supplied from any GPIO
    pub ext_rco_osc: bool,
    /// Enable the automatic RCO calibratio
    pub rco_calibration: bool,
}

/// External clock reference mode
#[derive(Copy, Clone, Debug, BitfieldSpecifier)]
#[bits = 1]
pub enum ExtRefMode {
    // Reference from XO circuit
    XO = 0,
    /// Reference from XIN pin
    XIN = 1,
}

impl radio::Register for XoRcoConf0 {
    const ADDRESS: u8 = Registers::XO_RCO_CONF0 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrConf3 {
    pub raw: u8,
}

impl radio::Register for RcoCalibrConf3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrConf2 {
    pub raw: u8,
}

impl radio::Register for RcoCalibrConf2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf4 {
    pub raw: u8,
}

impl radio::Register for PmConf4 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf3 {
    pub raw: u8,
}

impl radio::Register for PmConf3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf2 {
    pub raw: u8,
}

impl radio::Register for PmConf2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf1 {
    pub raw: u8,
}

impl radio::Register for PmConf1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct PmConf0 {
    pub raw: u8,
}

impl radio::Register for PmConf0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct McState1 {
    pub raw: u8,
}

impl radio::Register for McState1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct McState0 {
    pub raw: u8,
}

impl radio::Register for McState0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TxFifoStatus {
    pub raw: u8,
}

impl radio::Register for TxFifoStatus {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxFifoStatus {
    pub raw: u8,
}

impl radio::Register for RxFifoStatus {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrOut4 {
    pub raw: u8,
}

impl radio::Register for RcoCalibrOut4 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RcoCalibrOut3 {
    pub raw: u8,
}

impl radio::Register for RcoCalibrOut3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrOut1 {
    pub raw: u8,
}

impl radio::Register for VcoCalibrOut1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct VcoCalibrOut0 {
    pub raw: u8,
}

impl radio::Register for VcoCalibrOut0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct TxPcktInfo {
    pub raw: u8,
}

impl radio::Register for TxPcktInfo {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxPcktInfo {
    pub raw: u8,
}

impl radio::Register for RxPcktInfo {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct AfcCorr {
    pub raw: u8,
}

impl radio::Register for AfcCorr {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct LinkQualif2 {
    pub raw: u8,
}

impl radio::Register for LinkQualif2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct LinkQualif1 {
    pub raw: u8,
}

impl radio::Register for LinkQualif1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiLevel {
    pub raw: u8,
}

impl radio::Register for RssiLevel {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxPcktLen1 {
    pub raw: u8,
}

impl radio::Register for RxPcktLen1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxPcktLen0 {
    pub raw: u8,
}

impl radio::Register for RxPcktLen0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField3 {
    pub raw: u8,
}

impl radio::Register for CrcField3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField2 {
    pub raw: u8,
}

impl radio::Register for CrcField2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField1 {
    pub raw: u8,
}

impl radio::Register for CrcField1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct CrcField0 {
    pub raw: u8,
}

impl radio::Register for CrcField0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxAddreField1 {
    pub raw: u8,
}

impl radio::Register for RxAddreField1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RxAddreField0 {
    pub raw: u8,
}

impl radio::Register for RxAddreField0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct RssiLevelRun {
    pub raw: u8,
}

impl radio::Register for RssiLevelRun {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct DeviceInfo1 {
    pub part_no: u8,
}

impl radio::Register for DeviceInfo1 {
    const ADDRESS: u8 = Registers::DEVICE_INFO1 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct DeviceInfo0 {
    pub version: u8,
}

impl radio::Register for DeviceInfo0 {
    const ADDRESS: u8 = Registers::DEVICE_INFO0 as u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus3 {
    pub raw: u8,
}

impl radio::Register for IrqStatus3 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus2 {
    pub raw: u8,
}

impl radio::Register for IrqStatus2 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus1 {
    pub raw: u8,
}

impl radio::Register for IrqStatus1 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct IrqStatus0 {
    pub raw: u8,
}

impl radio::Register for IrqStatus0 {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}

#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct Fifo {
    pub raw: u8,
}

impl radio::Register for Fifo {
    const ADDRESS: u8 = 0u8;
    type Word = u8;
    type Error = Infallible;
}



//*******************************************************//

/// Switch mode power supply [SMPS] Voltage levels
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 3]
pub enum SMPSLevel {
    Smps1v2 = 0b001,
    Smps1v3 = 0b010,
    Smps1v4 = 0b011,
    Smps1v5 = 0b100,
    Smps1v6 = 0b101,
    Smps1v7 = 0b110,
    Smps1v8 = 0b111,
}

/// Clock Frequencies
#[derive(Copy, Clone, Debug)]
pub enum ClockFreq {
    Clock24MHz,
    Clock25MHz,
    Clock26MHz,
    Clock48MHz,
    Clock50MHz,
}

/// Supported frequency bands
pub const BANDS: [(u16, u16); 3] = [(430, 470), (470, 512), (860, 940)];

/// Check if a frequency is in one of the valid bands
pub fn frequency_valid(frequency: u16) -> bool {
    for (l, h) in BANDS.iter() {
        if frequency > *l && frequency < *h {
            return true;
        }
    }
    return false;
}

/// Clock recovery post filter length
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum PostFilterLen {
    FilterLen8 = 0x00,
    FilterLen16 = 0x01,
}

/// Packet handler packet format
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum PacketFormat {
    Basic = 0x00,
    FifteenFour = 0x01,
    UartOTA = 0x02,
    Stack = 0x03,
}

/// Packet handler receive mode
#[derive(Copy, Clone, PartialEq, Debug, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum RXMode {
    Normal = 0x00,
    DirectFIFO = 0x01,
    DirectGPIO = 0x02,
}



/// External SMPS mode
#[derive(Copy, Clone, Debug)]
pub enum ExtSmpsMode {
    Enable = 0x00,
    Disable = 0x20,
}

pub const PM_CONF0_EXT_SMPS_REGMASK: u8 = 0x20;

/// Radio states (table 48)
#[derive(Copy, Clone, PartialEq, Debug, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum State {
    Shutdown = 0xFF,
    Standby = 0x02,
    SleepA = 0x01, // Sleep without FIFO retention
    SleepB = 0x03, // Sleep with FIFO retention
    Ready = 0x00,
    Lock = 0x0C,
    Rx = 0x30,
    Tx = 0x5C,
    SynthSetup = 0x50,
}

impl radio::RadioState for State {
    fn idle() -> Self {
        State::Standby
    }

    fn sleep() -> Self {
        State::SleepA
    }
}

/// Radio commands (table 49)
#[derive(Copy, Clone, PartialEq, Debug, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum Command {
    Tx = 0x60,             // from: READY Send the S2-LP to TX state for transmission
    Rx = 0x61,             // from: READY Send the S2-LP to RX state for reception
    Ready = 0x62,          // from: STANDBY, SLEEP, LOCK Go to READY state
    Standby = 0x63,        // from: READY Go to STANDBY state
    Sleep = 0x64,          // from: READY Go to SLEEP state
    LockRx = 0x65, // from: READY Go to LOCK state by using the RX configuration of the synthesizer
    LockRx2 = 0x66, // from: READY Go to LOCK state by using the TX configuration of the synthesizer
    SAbort = 0x67, // from: TX, RX Exit from TX or RX states and go to READY state
    LDCReload = 0x68, // from: ANY Reload the LDC timer with a pre-programmed value storedin registers
    Reset = 0x70,     // from: ANY Reset the S2-LP state machine and registers values
    FlushRXFifo = 0x71, // from: All Clean the RX FIFO
    FlushTxFifo = 0x72, // from: All Clean the TX FIFO
    SequenceUpdate = 0x73, // from: TE ANY Reload the packet sequence counter with the value stored in register
}

/// S2-LP Register Addresses
#[derive(Copy, Clone, Debug)]
#[allow(non_camel_case_types)]
pub enum Registers {
    // GPIO configuration
    GPIO0_CONF = 0x00, // GPIO0 7:3 GPIO_SELECT, 2 RESERVED, 1:0 GPIO_MODE
    GPIO1_CONF = 0x01, // GPIO1 7:3 GPIO_SELECT, 2 RESERVED, 1:0 GPIO_MODE
    GPIO2_CONF = 0x02, // GPIO2 7:3 GPIO_SELECT, 2 RESERVED, 1:0 GPIO_MODE
    GPIO3_CONF = 0x03, // GPIO3 7:3 GPIO_SELECT, 2 RESERVED, 1:0 GPIO_MODE
    // Synthesizer configuration
    SYNT3 = 0x05,         // 7:0 SYNT[27:24] (MSB)
    SYNT2 = 0x06,         // 7:0 SYNT[23:16]
    SYNT1 = 0x07,         // 7:0 SYNT[15:8]
    SYNT0 = 0x08,         // 7:0 SYNT[7:0] (LSB)
    IF_OFFSET_ANA = 0x09, // 7:0 IF_OFFSET_ANA (default: 0x2A 300kHz)
    IF_OFFSET_DIG = 0x0A, // 7:0 IF_OFFSET_DIG (default: 0xB8 300kHz)
    // Channel configuration
    CH_SPACE = 0x0C, // 7:0 CH_SPACE (default: 0x3F)
    CH_NUM = 0x0D,   // 7:0 CH_NUM (defaults to 0x00)
    // Modulation options
    MOD4 = 0x0E, // 7:0 DATARATE_M[15:8] (default: 0x83)
    MOD3 = 0x0F, // 7:0 DATARATE_M[7:0] (default: 0x2B)
    MOD2 = 0x10, // 7:4 MOD_TYPE, 3:0 DATARATE_E (default 0x77)
    MOD1 = 0x11, // 7: PA_INTERP_EN, 6: MOD_INTERP_EN, 5:4 CONST_MAP, 3:0 FDEV_E
    MOD0 = 0x12, // 7:0 FDEV_M
    // Channel filter
    CHFLT = 0x13, // 7:4 CHFLT_M, 3:0 CHFLT_E
    // Automatic Frequency Compensation
    AFC2 = 0x14, // 7: AFC_FREEZE_ON_SYNC, 6: AFC_ENABLED, 5: AFC_MODE
    AFC1 = 0x15, // 7:0 AFC_FAST_PERIOD
    AFC0 = 0x16, // 7:4 AFC_FAST_GAIN 3:0 AFC_SLOW_GAIN
    // RSSI Filtering and Thresholding
    RSSI_FLT = 0x17, // 7:4 RSSI_FLT, 3:2 CS_MODE
    RSSI_TH = 0x18,  // 7:0 RSSI_TH
    // Automated gain control
    AGCCTRL4 = 0x1a, // 7:4 LOW_THRESHOLD_0, 3:0 LOW_THRESHOLD_1
    AGCCTRL3 = 0x1b, // 7:0 LOW_THRESHOLD_SEL
    AGCCTRL2 = 0x1c, // 5: FREEZE_ON_SYNC, 4: RESERVED, 3:0 MEAS_TIME
    AGCCTRL1 = 0x1d, // 7:4 HIGH_THRESHOLD 3:0 RESERVED
    AGCCTRL0 = 0x1e, // 7: AGC_ENABLE, 6: RESERVED 5:0 HOLD_TIME
    // Antenna switching
    ANT_SELECT_CONF = 0x1f, // 6:5 EQU_CTRL, 4: CS_BLANKING, 3: AS_ENABLE, 2:0 AS_MEAS_TIME
    // Clock recovery
    CLOCKREC2 = 0x20, // 7:5 CLK_REC_P_GAIN_SLOW, 4: CLK_REC_ALGO_SEL, 3:0 CLK_REC_I_GAIN_SLOW
    CLOCKREC1 = 0x21, // 7:5 CLK_REC_P_GAIN_FAST, 4: PSTFLT_LEN, 3:0 CLK_REC_I_GAIN_FAST
    // Packet control
    PCKTCTRL6 = 0x2B, // 7:2 SYNC_LEN, 1:0 PREAMBLE_LEN[9:8]
    PCKTCTRL5 = 0x2C, // 7:0 PREAMBLE_LEN[7:0]
    PCKTCTRL4 = 0x2D, // 7: LEN_WID, 6:4 RESERVED, 3: ADDRESS_LEN
    PCKTCTRL3 = 0x2E, // 7:6 PCKT_FRMT, 5:4 RX_MODE, 3: FSK4_SYM_SWAP, 2: BYTE_SWAP, 1:0 PREAMBLE_SEL
    PCKTCTRL2 = 0x2F, // 7:6 RESERVED, 5: FCS_TYPE_4G, 4: FEC_TYPE_4G/STOP_BIT, 3: INT_EN_4G/START_BIT, 2: MBUS_3OF6_EN, 1: MANCHESTER_EN 0: FIX_VAR_LEN
    PCKTCTRL1 = 0x30, // 7:5 CRC_MODE, 4: WHIT_EN, 3:2 TXSOURCE, 1: SECOND_SYNC_SEL, 0: FEC_EN,
    // Packet length
    PCKTLEN1 = 0x31, // 7:0 PCKTLEN1 (MSB)
    PCKTLEN0 = 0x32, // 7:0 PCKTLEN0 (LSB)
    // Sync word
    SYNC3 = 0x33, // 7:0 SYNC[3]
    SYNC2 = 0x34, // 7:0 SYNC[3]
    SYNC1 = 0x35, // 7:0 SYNC[3]
    SYNC0 = 0x36, // 7:0 SYNC[3]
    //
    QI = 0x37,          // 7:5 SQI_TH, 4:1 PQI_TH, 0 SQI_EN
    PCKT_PSTMBL = 0x38, // 7:0 PCKT_PSTMBL
    // Protocol configuration
    PROTOCOL2 = 0x39, // 7: CS_TIMEOUT_MASK, 6: SQI_TIMEOUT_MASK, 5: PQI_TIMEOUT_MASK, 4:3 TX_SEQ_NUM_RELOAD, 2: FIFO_GPIO_OUT_MUX_SEL, 1:0 LDC_TIMER_MULT
    PROTOCOL1 = 0x3A, // 7 LDC_MODE, 6 LDC_RELOAD_ON_SYNC, 5 PIGGYBACKING, 4 FAST_CS_TERM_EN, 3 SEED_RELOAD, 2 CSMA_ON, 1 CSMA_PERS_ON, 0 AUTO_PCKT_FLT
    PROTOCOL0 = 0x3B, // 7:4 NMAX_RETX, 3 NACK_TX, 2 AUTO_ACK, 1 PERS_RX
    // FIFO configuration
    FIFO_CONFIG3 = 0x3C, // 6:0 RX_AFTHR
    FIFO_CONFIG2 = 0x3D, // 6:0 RX_AETHR
    FIFO_CONFIG1 = 0x3E, // 6:0 TX_AFTHR
    FIFO_CONFIG0 = 0x3F, // 6:0 TX_AETHR
    // Packet filter options
    PCKT_FLT_OPTIONS = 0x40, // 6 RX_TIMEOUT_AND_OR_SEL, 4 SOURCE_ADDR_FLT, 3 DEST_VS_BROADCAST_ADDR, 2 DEST_VS_MULTICAST_ADDR, 1 DEST_VS_SOURCE_ADDR, 0 CRC_FLT
    PCKT_FLT_GOALS4 = 0x41,  // 7:0 RX_SOURCE_MASK
    PCKT_FLT_GOALS3 = 0x42,  // 7:0 RX_SOURCE_ADDR/DUAL_SYNC3
    PCKT_FLT_GOALS2 = 0x43,  // 7:0 BROADCAST_ADDR/DUAL_SYNC2
    PCKT_FLT_GOALS1 = 0x44,  // 7:0 MULTICAST_ADDR/DUAL_SYNC1
    PCKT_FLT_GOALS0 = 0x45,  // 7:0 TX_SOURCE_ADDR/DUAL_SYNC0
    // Timer options
    TIMERS5 = 0x46, // 7:0 RX_TIMER_CNTR
    TIMERS4 = 0x47, // 7:0 RX_TIMER_PRESC
    TIMERS3 = 0x48, // 7:0 LDC_TIMER_PRESC
    TIMERS2 = 0x49, // 7:0 LDC_TIMER_CNTR
    TIMERS1 = 0x4A, // 7:0 LDC_RELOAD_PRSC
    TIMERS0 = 0x4B, // 7:0 LDC_RELOAD_CNTR
    // CSMA configuration
    CSMA_CONF3 = 0x4C, // 7:0 BU_CNTR_SEED[14:8]
    CSMA_CONF2 = 0x4D, // 7:0 BU_CNTR_SEED[7:0]
    CSMA_CONF1 = 0x4E, // 7:2 BU_PRSC, 1:0 CCA_PERIOD
    CSMA_CONF0 = 0x4F, // 7:4 CCA_LEN, 3 RESERVED, 2:0 NBACKOFF_MAX
    // Interrupt configuration
    IRQ_MASK3 = 0x50, // 7:0 INT_MASK[31:24]
    IRQ_MASK2 = 0x51, // 7:0 INT_MASK[23:16]
    IRQ_MASK1 = 0x52, // 7:0 INT_MASK[15:8]
    IRQ_MASK0 = 0x53, // 7:0 INT_MASK[7:0]
    // RX timer configuration
    FAST_RX_TIMER = 0x54, // 7:0 RSSI_SETTLING_LIdMIT
    // Power amplifier configuration
    PA_POWER8 = 0x5A,  // 6:0 PA_LEVEL8
    PA_POWER7 = 0x5B,  // 6:0 PA_LEVEL7
    PA_POWER6 = 0x5C,  // 6:0 PA_LEVEL6
    PA_POWER5 = 0x5D,  // 6:0 PA_LEVEL5
    PA_POWER4 = 0x5E,  // 6:0 PA_LEVEL4
    PA_POWER3 = 0x5F,  // 6:0 PA_LEVEL3
    PA_POWER2 = 0x60,  // 6:0 PA_LEVEL2
    PA_POWER1 = 0x61,  // 6:0 PA_LEVEL1
    PA_POWER0 = 0x62, // 7 DIG_SMOOTH_EN, 6 PA_MAXDBM, 5 PA_RAMP_EN, 4:3 PA_RAMP_STEP_LEN, 2:0 PA_LEVEL_MAX_IDX
    PA_CONFIG1 = 0x63, // 3:2 FIR_CFG, 1 FIR_EN
    PA_CONFIG0 = 0x64, // 1:0 PA_FC
    // Synthesizer configuration
    SYNTH_CONFIG2 = 0x65,  // 2 PLL_PFD_SPLIT_EN
    VCO_CONFIG = 0x68,     // 5 VCO_CALAMP_EXT_SEL, 4 VCO_CALFREQ_EXT_SEL
    VCO_CALIBR_IN2 = 0x69, // RESERVED
    VCO_CALIBR_IN1 = 0x6A, // RESERVED
    VCO_CALIBR_IN0 = 0x6B, // RESERVED
    // Oscillator configuration
    XO_RCO_CONF1 = 0x6C,     // 4 PD_CLKDIV
    XO_RCO_CONF0 = 0x6D,     // 7 EXT_REF, 5:4 GM_CONF, 3 REFDIV, 1 EXT_RCO_OSC, 0 RCO_CALIBRATION
    RCO_CALIBR_CONF3 = 0x6E, // 7:4 RWT_IN, 3:0 RFB_IN[4:1]
    RCO_CALIBR_CONF2 = 0x6F, // 7 RFB_IN[0]
    // Power configuration
    PM_CONF4 = 0x75, // 5 EXT_SMPS
    PM_CONF3 = 0x76, // 7 KRM_EN, 6:0 KRM[14:8]
    PM_CONF2 = 0x77, // 7:0 KRM[7:0]
    PM_CONF1 = 0x78, // 6 BATTERY_LVL_EN, 5:4 SET_BLD_TH, 3 SMPS_LVL_MODE, 2 BYPASS_LDO
    PM_CONF0 = 0x79, // 6:4 SET_SMPS_LVL, 0 SLEEP_MODE_SEL
    // Status registeres
    MC_STATE1 = 0x8D, // 4 RCO_CAL_OK, 3 ANT_SEL, 2 TX_FIFO_FULL, 1 RX_FIFO_EMPTY, 0 ERROR_LOCK
    MC_STATE0 = 0x8E, // 7:1 STATE, 0 XO_ON,
    TX_FIFO_STATUS = 0x8F, // 7:0 NELEM_TXFIFO
    RX_FIFO_STATUS = 0x90, // 7:0 NELEM_RXFIFO
    // Radio calibration outputs
    RCO_CALIBR_OUT4 = 0x94, // 7:4 RWT_OUT, 3:0 RFB_OUT[4:1]
    RCO_CALIBR_OUT3 = 0x95, // 7 RFB_OUT[0],
    VCO_CALIBR_OUT1 = 0x99, // 3:0 VCO_CAL_AMP_OUT
    VCO_CALIBR_OUT0 = 0x9A, // 6:0 VCO_CAL_FREQ_OUT
    // Packet informatino
    TX_PCKT_INFO = 0x9C, // 5:4 TX_SEQ_NUM, 3:0 N_RETX
    RX_PCKT_INFO = 0x9D, // 2 NACK_RX, 1:0 RX_SEQ_NUM
    // Link information
    AFC_CORR = 0x9E,     // 7:0 AFC_CORR
    LINK_QUALIF2 = 0x9F, // 7:0 PQI
    LINK_QUALIF1 = 0xA0, // 7 CS
    RSSI_LEVEL = 0xA2,   // 7:0 RSSI_LEVEL
    // Packet lengths
    RX_PCKT_LEN1 = 0xA4, // 7:0 RX_PCKT_LEN[14:8]
    RX_PCKT_LEN0 = 0xA5, // 7:0 RX_PCKT_LEN[7:0]
    // Packet CRC
    CRC_FIELD3 = 0xA6, // 7:0 CRC_FIELD3
    CRC_FIELD2 = 0xA7, // 7:0 CRC_FIELD2
    CRC_FIELD1 = 0xA8, // 7:0 CRC_FIELD1
    CRC_FIELD0 = 0xA9, // 7:0 CRC_FIELD0
    // Received packet address
    RX_ADDRE_FIELD1 = 0xAA, // 7:0 RX_ADDRE_FIELD1
    RX_ADDRE_FIELD0 = 0xAB, // 7:0 RX_ADDRE_FIELD0
    RSSI_LEVEL_RUN = 0xEF,  // 7:0 RSSI_LEVEL_RUN
    // Device info
    DEVICE_INFO1 = 0xF0, // 7:0 PARTNUM
    DEVICE_INFO0 = 0xF1, // 7:0 VERSION
    // Interrupt status
    IRQ_STATUS3 = 0xFA, // 7:0 INT_LEVEL[31:24]
    IRQ_STATUS2 = 0xFB, // 7:0 INT_LEVEL[23:16]
    IRQ_STATUS1 = 0xFC, // 7:0 INT_LEVEL[15:8]
    IRQ_STATUS0 = 0xFD, // 7:0 INT_LEVEL[7:0]

    FIFO = 0xFF, // FIFO Read/Write address
}

