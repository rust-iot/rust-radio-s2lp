
//! S2-LP Configuration and Helpers
//! See also:
//! https://github.com/ryankurte/libsts2lp/blob/master/S2LP_Library/Src/S2LP_Radio.c
//! Copyright 2018 Ryan Kurte

const MAX_PA_VALUE: usize = 14;
const MIN_PA_VALUE: usize = 31;

/// VCO center frequency in Hz
const VCO_CENTER_FREQ: usize = 3600000000;

/// Band select factor for high band. Factor B in the equation 2
const HIGH_BAND_FACTOR: usize = 4;
/// Band select factor for middle band. Factor B in the equation 2
const MIDDLE_BAND_FACTOR: usize = 8;

/// Lower limit of the high band: 860 MHz (S2-LPQTR)
const HIGH_BAND_LOWER_LIMIT: usize = 825900000;
/// Upper limit of the high band: 940 MHz (S2-LPCBQTR)  
const HIGH_BAND_UPPER_LIMIT: usize = 1056000000;
/// Lower limit of the middle band: 430 MHz (S2-LPQTR)  
const MIDDLE_BAND_LOWER_LIMIT: usize = 412900000;
/// Upper limit of the middle band: 470 MHz (S2-LPCBQTR)
const MIDDLE_BAND_UPPER_LIMIT: usize = 527100000;

/// Minimum datarate supported by S2LP 100 bps
const MINIMUM_DATARATE: usize = 100;

/// Maximum datarate supported by S2LP 250 ksps
const MAXIMUM_DATARATE: usize = 250000;


