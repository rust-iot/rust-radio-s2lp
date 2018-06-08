// S2-LP Device definitions
// Copyright 2018 Ryan Kurte

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

#[doc="Clock Frequencies"]
pub enum ClockFreq {
    Clock24MHz,
    Clock25MHz,
    Clock26MHz,
    Clock48MHz,
    Clock50MHz,
}

#[doc="Supported frequency bands"]
pub const BANDS: [(u16, u16); 3] = [(430, 470), (470, 512), (860, 940)];

#[doc="Modulation modes"]
pub enum Modulation {
    Mod2FSK   = 0b000,
    Mod4FSK   = 0b001,
    Mod2GFSK  = 0b010,
    Mod4GFSK  = 0b011,
    ModASK    = 0b101,
    ModDirect = 0b110,
    ModCW     = 0b111,
}

#[doc="GPIO Pin Modes"]
pub enum GPIOMode {
    DigitalInput = 0b01,
    DigitalOutputLowPower = 0b00,
    DigitalOutputHighPower = 0b11,
}

#[doc="GPIO Pin Operation"]
pub enum GPIOSelect {
    
}

#[doc="Clock recovery post filter length"]
pub enum PostFilterLen {
    FilterLen8 = 0x00,
    FilterLen16 = 0x01,
}

#[doc="Packet handler packet format"]
pub enum PacketFormat {
    Basic = 0x00,
    FifteenFour = 0x01,
    UartOTA = 0x02,
    Stack = 0x03,
}

#[doc="Packet handler receive mode"]
pub enum RXMode {
    Normal = 0x00,
    DirectFIFO = 0x01,
    DirectGPIO = 0x02,
}

#[doc="Radio states (table 48)"]
pub enum State {
    Shutdown = 0xFF,
    Standby = 0x02,
    SleepA = 0x01,  // Sleep without FIFO retention
    SleepB = 0x03,  // Sleep with FIFO retention
    Ready = 0x00,
    Lock = 0x0C,
    Rx = 0x30,
    Tx = 0x5C,
    SynthSetup = 0x50,
}

#[doc="Radio commands (table 49)"]
pub enum Command {
    Tx              = 0x60, // from: READY Send the S2-LP to TX state for transmission
    Rx              = 0x61, // from: READY Send the S2-LP to RX state for reception
    Ready           = 0x62, // from: STANDBY, SLEEP, LOCK Go to READY state
    Standby         = 0x63, // from: READY Go to STANDBY state
    Sleep           = 0x64, // from: READY Go to SLEEP state
    LockRx          = 0x65, // from: READY Go to LOCK state by using the RX configuration of the synthesizer
    LockRx          = 0x66, // from: READY Go to LOCK state by using the TX configuration of the synthesizer
    SAbort          = 0x67, // from: TX, RX Exit from TX or RX states and go to READY state
    LDCReload       = 0x68, // from: ANY Reload the LDC timer with a pre-programmed value storedin registers
    Reset           = 0x70, // from: ANY Reset the S2-LP state machine and registers values
    FlushRXFifo     = 0x71, // from: All Clean the RX FIFO
    FlushTxFifo     = 0x72, // from: All Clean the TX FIFO
    SequenceUpdate  = 0x73, // from: TE ANY Reload the packet sequence counter with the value stored in register
}

#[doc="S2-LP Register Locations"]
#[allow(non_camel_case_types)]
pub enum Registers {
    // GPIO configuration
    GPIO0_CONF      = 0x00, // GPIO0 7:3 GPIO_SELECT, 2 RESERVED, 1:0 GPIO_MODE 
    GPIO1_CONF      = 0x01, // GPIO1 7:3 GPIO_SELECT, 2 RESERVED, 1:0 GPIO_MODE 
    GPIO2_CONF      = 0x02, // GPIO2 7:3 GPIO_SELECT, 2 RESERVED, 1:0 GPIO_MODE 
    GPIO3_CONF      = 0x03, // GPIO3 7:3 GPIO_SELECT, 2 RESERVED, 1:0 GPIO_MODE 
    // Synthesizer configuration
    SYNT3           = 0x05, // 7:0 SYNT[27:24] (MSB)
    SYNT2           = 0x06, // 7:0 SYNT[23:16]
    SYNT1           = 0x07, // 7:0 SYNT[15:8]
    SYNT0           = 0x08, // 7:0 SYNT[7:0] (LSB)
    IF_OFFSET_ANA   = 0x09, // 7:0 IF_OFFSET_ANA (default: 0x2A 300kHz)
    IF_OFFSET_DIG   = 0x0A, // 7:0 IF_OFFSET_DIG (default: 0xB8 300kHz)
    // Channel configuration
    CHSPACE         = 0x0C, // 7:0 CH_SPACE (default: 0x3F)
    CHNUM           = 0x0D, // 7:0 CH_NUM (defaults to 0x00)
    // Modulation options
    MOD4            = 0x0E, // 7:0 DATARATE_M[15:8] (default: 0x83)
    MOD3            = 0x0F, // 7:0 DATARATE_M[7:0] (default: 0x2B)
    MOD2            = 0x10, // 7:4 MOD_TYPE, 3:0 DATARATE_E (default 0x77)
    MOD1            = 0x11, // 7: PA_INTERP_EN, 6: MOD_INTERP_EN, 5:4 CONST_MAP, 3:0 FDEV_E
    MOD0            = 0x12, // 7:0 FDEV_M
    // Channel filter
    CHFLT           = 0x13, // 7:4 CHFLT_M, 3:0 CHFLT_E
    // Automatic Frequency Compensation
    AFC2            = 0x14, // 7: AFC_FREEZE_ON_SYNC, 6: AFC_ENABLED, 5: AFC_MODE
    AFC1            = 0x15, // 7:0 AFC_FAST_PERIOD
    AFC0            = 0x16, // 7:4 AFC_FAST_GAIN 3:0 AFC_SLOW_GAIN
    // RSSI Filtering and Thresholding
    RSSI_FLT        = 0x17, // 7:4 RSSI_FLT, 3:2 CS_MODE
    RSSI_TH         = 0x18, // 7:0 RSSI_TH
    // Automated gain control
    AGCCTRL4        = 0x1a, // 7:4 LOW_THRESHOLD_0, 3:0 LOW_THRESHOLD_1
    AGCCTRL3        = 0x1b, // 7:0 LOW_THRESHOLD_SEL 
    AGCCTRL2        = 0x1c, // 5: FREEZE_ON_SYNC, 4: RESERVED, 3:0 MEAS_TIME
    AGCCTRL1        = 0x1d, // 7:4 HIGH_THRESHOLD 3:0 RESERVED
    AGCCTRL0        = 0x1e, // 7: AGC_ENABLE, 6: RESERVED 5:0 HOLD_TIME
    // Antenna switching
    ANT_SELECT_CONF = 0x1f, // 6:5 EQU_CTRL, 4: CS_BLANKING, 3: AS_ENABLE, 2:0 AS_MEAS_TIME
    // Clock recovery
    CLOCKREC2       = 0x20, // 7:5 CLK_REC_P_GAIN_SLOW, 4: CLK_REC_ALGO_SEL, 3:0 CLK_REC_I_GAIN_SLOW
    CLOCKREC1       = 0x21, // 7:5 CLK_REC_P_GAIN_FAST, 4: PSTFLT_LEN, 3:0 CLK_REC_I_GAIN_FAST
    // Packet control
    PCKTCTRL6       = 0x2B, // 7:2 SYNC_LEN, 1:0 PREAMBLE_LEN[9:8]
    PCKTCTRL5       = 0x2C, // 7:0 PREAMBLE_LEN[7:0]
    PCKTCTRL4       = 0x2D, // 7: LEN_WID, 6:4 RESERVED, 3: ADDRESS_LEN
    PCKTCTRL3       = 0x2E, // 7:6 PCKT_FRMT, 5:4 RX_MODE, 3: FSK4_SYM_SWAP, 2: BYTE_SWAP, 1:0 PREAMBLE_SEL
    PCKTCTRL2       = 0x2F, // 7:6 RESERVED, 5: FCS_TYPE_4G, 4: FEC_TYPE_4G/STOP_BIT, 3: INT_EN_4G/START_BIT, 2: MBUS_3OF6_EN, 1: MANCHESTER_EN 0: FIX_VAR_LEN
    PCKTCTRL1       = 0x30, // 7:5 CRC_MODE, 4: WHIT_EN, 3:2 TXSOURCE, 1: SECOND_SYNC_SEL, 0: FEC_EN,
    // Packet length
    PCKTLEN1        = 0x31, // 7:0 PCKTLEN1 (MSB)
    PCKTLEN0        = 0x32, // 7:0 PCKTLEN0 (LSB)
    // Sync word
    SYNC3           = 0x33, // 7:0 SYNC[3]
    SYNC2           = 0x34, // 7:0 SYNC[3]
    SYNC1           = 0x35, // 7:0 SYNC[3]
    SYNC0           = 0x36, // 7:0 SYNC[3]
    //
    QI              = 0x37, // 7:5 SQI_TH, 4:1 PQI_TH, 0 SQI_EN
    PCKT_PSTMBL     = 0x38, // 7:0 PCKT_PSTMBL
    // Protocol configuration
    PROTOCOL2       = 0x39, // 7: CS_TIMEOUT_MASK, 6: SQI_TIMEOUT_MASK, 5: PQI_TIMEOUT_MASK, 4:3 TX_SEQ_NUM_RELOAD, 2: FIFO_GPIO_OUT_MUX_SEL, 1:0 LDC_TIMER_MULT
    PROTOCOL1       = 0x3A, // 7 LDC_MODE, 6 LDC_RELOAD_ON_SYNC, 5 PIGGYBACKING, 4 FAST_CS_TERM_EN, 3 SEED_RELOAD, 2 CSMA_ON, 1 CSMA_PERS_ON, 0 AUTO_PCKT_FLT
    PROTOCOL0       = 0x3B, // 7:4 NMAX_RETX, 3 NACK_TX, 2 AUTO_ACK, 1 PERS_RX
    // FIFO configuration 
    FIFO_CONFIG3    = 0x3C, // 6:0 RX_AFTHR
    FIFO_CONFIG2    = 0x3D, // 6:0 RX_AETHR
    FIFO_CONFIG1    = 0x3E, // 6:0 TX_AFTHR
    FIFO_CONFIG0    = 0x3F, // 6:0 TX_AETHR
    // Packet filter options
    PCKT_FLT_OPTIONS = 0x40, // 6 RX_TIMEOUT_AND_OR_SEL, 4 SOURCE_ADDR_FLT, 3 DEST_VS_BROADCAST_ADDR, 2 DEST_VS_MULTICAST_ADDR, 1 DEST_VS_SOURCE_ADDR, 0 CRC_FLT
    PCKT_FLT_GOALS4 = 0x41, // 7:0 RX_SOURCE_MASK
    PCKT_FLT_GOALS3 = 0x42, // 7:0 RX_SOURCE_ADDR/DUAL_SYNC3
    PCKT_FLT_GOALS2 = 0x43, // 7:0 BROADCAST_ADDR/DUAL_SYNC2
    PCKT_FLT_GOALS1 = 0x44, // 7:0 MULTICAST_ADDR/DUAL_SYNC1
    PCKT_FLT_GOALS0 = 0x45, // 7:0 TX_SOURCE_ADDR/DUAL_SYNC0
    // Timer options
    TIMERS5         = 0x46, // 7:0 RX_TIMER_CNTR
    TIMERS4         = 0x47, // 7:0 RX_TIMER_PRESC
    TIMERS3         = 0x48, // 7:0 LDC_TIMER_PRESC
    TIMERS2         = 0x49, // 7:0 LDC_TIMER_CNTR
    TIMERS1         = 0x4A, // 7:0 LDC_RELOAD_PRSC
    TIMERS0         = 0x4B, // 7:0 LDC_RELOAD_CNTR
    // CSMA configuration
    CSMA_CONF3      = 0x4C, // 7:0 BU_CNTR_SEED[14:8]
    CSMA_CONF2      = 0x4D, // 7:0 BU_CNTR_SEED[7:0]
    CSMA_CONF1      = 0x4E, // 7:2 BU_PRSC, 1:0 CCA_PERIOD
    CSMA_CONF0      = 0x4F, // 7:4 CCA_LEN, 3 RESERVED, 2:0 NBACKOFF_MAX
    // Interrupt configuration
    IRQ_MASK3       = 0x50, // 7:0 INT_MASK[31:24]
    IRQ_MASK2       = 0x51, // 7:0 INT_MASK[23:16]
    IRQ_MASK1       = 0x52, // 7:0 INT_MASK[15:8]
    IRQ_MASK0       = 0x53, // 7:0 INT_MASK[7:0]
    // RX timer configuration
    FAST_RX_TIMER   = 0x54, // 7:0 RSSI_SETTLING_LIdMIT
    // Power amplifier configuration
    PA_POWER8       = 0x5A, // 6:0 PA_LEVEL8
    PA_POWER7       = 0x5B, // 6:0 PA_LEVEL7
    PA_POWER6       = 0x5C, // 6:0 PA_LEVEL6
    PA_POWER5       = 0x5D, // 6:0 PA_LEVEL5
    PA_POWER4       = 0x5E, // 6:0 PA_LEVEL4
    PA_POWER3       = 0x5F, // 6:0 PA_LEVEL3
    PA_POWER2       = 0x60, // 6:0 PA_LEVEL2
    PA_POWER1       = 0x61, // 6:0 PA_LEVEL1
    PA_POWER0       = 0x62, // 7 DIG_SMOOTH_EN, 6 PA_MAXDBM, 5 PA_RAMP_EN, 4:3 PA_RAMP_STEP_LEN, 2:0 PA_LEVEL_MAX_IDX
    PA_CONFIG1      = 0x63, // 3:2 FIR_CFG, 1 FIR_EN
    PA_CONFIG0      = 0x64, // 1:0 PA_FC
    // Synthesizer configuration
    SYNTH_CONFIG2   = 0x65, // 2 PLL_PFD_SPLIT_EN
    VCO_CONFIG      = 0x68, // 5 VCO_CALAMP_EXT_SEL, 4 VCO_CALFREQ_EXT_SEL
    VCO_CALIBR_IN2  = 0x69, // RESERVED
    VCO_CALIBR_IN1  = 0x6A, // RESERVED
    VCO_CALIBR_IN0  = 0x6B, // RESERVED
    // Oscillator configuration
    XO_RCO_CONF1    = 0x6C, // 4 PD_CLKDIV
    XO_RCO_CONF0    = 0x6D, // 7 EXT_REF, 5:4 GM_CONF, 3 REFDIV, 1 EXT_RCO_OSC, 0 RCO_CALIBRATION
    RCO_CALIBR_CONF3 = 0x6E, // 7:4 RWT_IN, 3:0 RFB_IN[4:1]
    RCO_CALIBR_CONF2 = 0x6F, // 7 RFB_IN[0]
    // Power configuration
    PM_CONF4        = 0x75, // 5 EXT_SMPS
    PM_CONF3        = 0x76, // 7 KRM_EN, 6:0 KRM[14:8]
    PM_CONF2        = 0x77, // 7:0 KRM[7:0]
    PM_CONF1        = 0x78, // 6 BATTERY_LVL_EN, 5:4 SET_BLD_TH, 3 SMPS_LVL_MODE, 2 BYPASS_LDO
    PM_CONF0        = 0x79, // 6:4 SET_SMPS_LVL, 0 SLEEP_MODE_SEL
    // Status registeres
    MC_STATE1       = 0x8D, // 4 RCO_CAL_OK, 3 ANT_SEL, 2 TX_FIFO_FULL, 1 RX_FIFO_EMPTY, 0 ERROR_LOCK
    MC_STATE0       = 0x8E, // 7:1 STATE, 0 XO_ON,
    TX_FIFO_STATUS  = 0x8F, // 7:0 NELEM_TXFIFO
    RX_FIFO_STATUS  = 0x90, // 7:0 NELEM_RXFIFO
    // Radio calibration outputs
    RCO_CALIBR_OUT4 = 0x94, // 7:4 RWT_OUT, 3:0 RFB_OUT[4:1]
    RCO_CALIBR_OUT3 = 0x95, // 7 RFB_OUT[0], 
    VCO_CALIBR_OUT1 = 0x99, // 3:0 VCO_CAL_AMP_OUT
    VCO_CALIBR_OUT0 = 0x9A, // 6:0 VCO_CAL_FREQ_OUT
    // Packet informatino
    TX_PCKT_INFO    = 0x9C, // 5:4 TX_SEQ_NUM, 3:0 N_RETX
    RX_PCKT_INFO    = 0x9D, // 2 NACK_RX, 1:0 RX_SEQ_NUM
    // Link information
    AFC_CORR        = 0x9E, // 7:0 AFC_CORR
    LINK_QUALIF2    = 0x9F, // 7:0 PQI
    LINK_QUALIF1    = 0xA0, // 7 CS
    RSSI_LEVEL      = 0xA2, // 7:0 RSSI_LEVEL
    // Packet lengths
    RX_PCKT_LEN1    = 0xA4, // 7:0 RX_PCKT_LEN[14:8]
    RX_PCKT_LEN0    = 0xA5, // 7:0 RX_PCKT_LEN[7:0]
    // Packet CRC
    CRC_FIELD3      = 0xA6, // 7:0 CRC_FIELD3
    CRC_FIELD2      = 0xA7, // 7:0 CRC_FIELD2
    CRC_FIELD1      = 0xA8, // 7:0 CRC_FIELD1
    CRC_FIELD0      = 0xA9, // 7:0 CRC_FIELD0
    // Received packet address
    RX_ADDRE_FIELD1 = 0xAA, // 7:0 RX_ADDRE_FIELD1
    RX_ADDRE_FIELD0 = 0xAB, // 7:0 RX_ADDRE_FIELD0
    RSSI_LEVEL_RUN  = 0xEF, // 7:0 RSSI_LEVEL_RUN
    // Device info
    DEVICE_INFO1    = 0xF0, // 7:0 PARTNUM
    DEVICE_INFO0    = 0xF1, // 7:0 VERSION
    // Interrupt status
    IRQ_STATUS3     = 0xFA, // 7:0 INT_LEVEL[31:24]
    IRQ_STATUS2     = 0xFB, // 7:0 INT_LEVEL[23:16]
    IRQ_STATUS1     = 0xFC, // 7:0 INT_LEVEL[15:8]
    IRQ_STATUS0     = 0xFD, // 7:0 INT_LEVEL[7:0]
}