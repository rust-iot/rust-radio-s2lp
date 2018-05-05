

#[doc="Switch mode power supply [SMPS] Voltage levels"]
pub enum SMPSLevel {
    Smps1v2 = 0b001,
    Smps1v3 = 0b010,
    Smps1v4 = 0b011,
    Smps1v5 = 0b100,
    Smps1v6 = 0b101,
    Smps1v7 = 0b110,
    Smps1v8 = 0b111,
}

pub enum ClockFreq {
    Clock24MHz,
    Clock25MHz,
    Clock26MHz,
    Clock48MHz,
    Clock50MHz,
}

pub const Bands: [(u16, u16); 3] = [(430, 470), (470, 512), (860, 940)];

pub enum Modulation {
    Mod2FSK   = 0b000,
    Mod4FSK   = 0b001,
    Mod2GFSK  = 0b010,
    Mod4GFSK  = 0b011,
    ModASK    = 0b101,
    ModDirect = 0b110,
    ModCW     = 0b111,
}