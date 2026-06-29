//! Register addresses, bit flags, and constants for iarduino I2C motor controller.
//!
//! Maps directly to `iarduino_I2C_Motor.h` definitions.

use core::fmt;

// ── Chip identification ───────────────────────────────────────────────────────

/// Chip ID for Flash series.
pub const DEF_CHIP_ID_FLASH: u8 = 0x3C;
/// Chip ID for Metro series.
pub const DEF_CHIP_ID_METRO: u8 = 0xC3;
/// Module model identifier.
pub const DEF_MODEL_MOT: u8 = 0x14;

// ── Register addresses ────────────────────────────────────────────────────────

/// Flags register `REG_FLAGS_0` (read).
pub const REG_FLAGS_0: u8 = 0x00;
/// Bits register `REG_BITS_0` (read/write).
pub const REG_BITS_0: u8 = 0x01;
/// Flags register `REG_FLAGS_1` (read).
#[allow(dead_code)]
pub const REG_FLAGS_1: u8 = 0x02;
/// Bits register `REG_BITS_1` (read/write).
#[allow(dead_code)]
pub const REG_BITS_1: u8 = 0x03;
/// Module model number.
pub const REG_MODEL: u8 = 0x04;
/// Firmware version.
#[allow(dead_code)]
pub const REG_VERSION: u8 = 0x05;
/// I2C address register.
pub const REG_ADDRESS: u8 = 0x06;
/// Chip ID register.
#[allow(dead_code)]
pub const REG_CHIP_ID: u8 = 0x07;
/// PWM frequency (low byte).
pub const REG_MOT_FREQUENCY_L: u8 = 0x08;
/// PWM frequency (high byte).
#[allow(dead_code)]
pub const REG_MOT_FREQUENCY_H: u8 = 0x09;
/// Max RPM deviation percent.
pub const REG_MOT_MAX_RPM_DEV: u8 = 0x0A;
/// Manufacturer data (also used by `saveManufacturer`).
pub const REG_MANUFACTURER: u8 = 0x0B;
/// Status flags register (`MOT_FLG_*`).
pub const REG_MOT_FLG: u8 = 0x10;
/// Hall sensor magnet count.
pub const REG_MOT_MAGNET: u8 = 0x11;
/// Gear ratio (low byte).
pub const REG_MOT_REDUCER_L: u8 = 0x12;
/// Gear ratio (center byte).
#[allow(dead_code)]
pub const REG_MOT_REDUCER_C: u8 = 0x13;
/// Gear ratio (high byte).
#[allow(dead_code)]
pub const REG_MOT_REDUCER_H: u8 = 0x14;
/// Set PWM value (low byte).
pub const REG_MOT_SET_PWM_L: u8 = 0x15;
/// Set PWM value (high byte).
#[allow(dead_code)]
pub const REG_MOT_SET_PWM_H: u8 = 0x16;
/// Set RPM value (low byte).
pub const REG_MOT_SET_RPM_L: u8 = 0x17;
/// Set RPM value (high byte).
#[allow(dead_code)]
pub const REG_MOT_SET_RPM_H: u8 = 0x18;
/// Get actual RPM (low byte).
pub const REG_MOT_GET_RPM_L: u8 = 0x19;
/// Get actual RPM (high byte).
#[allow(dead_code)]
pub const REG_MOT_GET_RPM_H: u8 = 0x1A;
/// Total revolutions (low byte).
pub const REG_MOT_GET_REV_L: u8 = 0x1B;
/// Total revolutions (center byte).
#[allow(dead_code)]
pub const REG_MOT_GET_REV_C: u8 = 0x1C;
/// Total revolutions (high byte).
#[allow(dead_code)]
pub const REG_MOT_GET_REV_H: u8 = 0x1D;
/// Stop after revolutions (low byte).
pub const REG_MOT_STOP_REV_L: u8 = 0x1E;
/// Stop after revolutions (center byte).
#[allow(dead_code)]
pub const REG_MOT_STOP_REV_C: u8 = 0x1F;
/// Stop after revolutions (high byte).
#[allow(dead_code)]
pub const REG_MOT_STOP_REV_H: u8 = 0x20;
/// Stop after time (low byte).
pub const REG_MOT_STOP_TMR_L: u8 = 0x21;
/// Stop after time (center byte).
#[allow(dead_code)]
pub const REG_MOT_STOP_TMR_C: u8 = 0x22;
/// Stop after time (high byte).
#[allow(dead_code)]
pub const REG_MOT_STOP_TMR_H: u8 = 0x23;
/// Stop control register.
pub const REG_MOT_STOP: u8 = 0x24;
/// Bits register `REG_BITS_2` (direction, inversion).
pub const REG_BITS_2: u8 = 0x25;
/// Nominal motor voltage (tenths of volt).
pub const REG_MOT_VOLTAGE: u8 = 0x26;
/// Nominal RPM (low byte).
pub const REG_MOT_NOMINAL_RPM_L: u8 = 0x27;
/// Nominal RPM (high byte).
#[allow(dead_code)]
pub const REG_MOT_NOMINAL_RPM_H: u8 = 0x28;

// ── Flag bits in REG_FLAGS_0 ──────────────────────────────────────────────────

/// Reset flag (indicates reset complete).
pub const FLG_RESET: u8 = 0b1000_0000;
/// I2C pull-up support flag.
pub const FLG_I2C_UP: u8 = 0b0000_0100;

// ── Control bits in REG_BITS_0 ────────────────────────────────────────────────

/// Set reset bit.
pub const BIT_SET_RESET: u8 = 0b1000_0000;
/// Block address change.
pub const BIT_BLOCK_ADR: u8 = 0b0000_1000;
/// Enable address save to flash.
pub const BIT_SAVE_ADR_EN: u8 = 0b0000_0010;
/// Set I2C pull-up.
pub const BIT_SET_I2C_UP: u8 = 0b0000_0100;

// ── Flag bits in REG_MOT_FLG ──────────────────────────────────────────────────

/// Speed set by RPM flag.
#[allow(dead_code)]
pub const MOT_FLG_RPM_EN: u8 = 0x80;
/// RPM error flag (deviation exceeded threshold).
pub const MOT_FLG_RPM_ERR: u8 = 0x20;
/// Driver error flag (overcurrent, overtemperature, undervoltage).
pub const MOT_FLG_DRV_ERR: u8 = 0x10;
/// Motor stopped flag.
pub const MOT_FLG_STOP: u8 = 0x02;
/// Motor neutral (free-wheeling) flag.
pub const MOT_FLG_NEUTRAL: u8 = 0x01;

// ── Control bits in REG_MOT_STOP ──────────────────────────────────────────────

/// Stop motor bit.
pub const MOT_BIT_STOP: u8 = 0x02;
/// Release motor outputs at stop bit.
pub const MOT_BIT_NEUTRAL: u8 = 0x01;

// ── Control bits in REG_BITS_2 ────────────────────────────────────────────────

/// Clockwise direction at positive speed.
pub const MOT_BIT_DIR_CKW: u8 = 0x04;
/// Reducer inversion flag.
pub const MOT_BIT_INV_RDR: u8 = 0x02;
/// Motor pin inversion flag.
pub const MOT_BIT_INV_PIN: u8 = 0x01;

// ── Error return values ───────────────────────────────────────────────────────

/// Speed error code (returned by `error_flags()`).
pub const MOT_ERR_SPD: u8 = 1;
/// Driver error code (returned by `error_flags()`).
pub const MOT_ERR_DRV: u8 = 2;

// ── Public enums ──────────────────────────────────────────────────────────────

/// Speed value type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpeedType {
    /// Meters per second.
    MPerSec,
    /// Revolutions per minute.
    Rpm,
    /// PWM duty cycle (0-100%).
    Pwm,
}

/// Stop condition type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StopType {
    /// Immediate stop.
    Immediate,
    /// Distance in meters.
    Meters,
    /// Number of full revolutions.
    Revolutions,
    /// Time in seconds.
    Seconds,
}

#[allow(dead_code)]
impl SpeedType {
    /// Convert to the u8 constant used by the device protocol.
    pub(crate) const fn to_u8(self) -> u8 {
        match self {
            SpeedType::MPerSec => 5,
            SpeedType::Rpm => 7,
            SpeedType::Pwm => 8,
        }
    }
}

#[allow(dead_code)]
impl StopType {
    /// Convert to the u8 constant used by the device protocol.
    pub(crate) const fn to_u8(self) -> u8 {
        match self {
            StopType::Immediate => 0xFF,
            StopType::Meters => 3,
            StopType::Revolutions => 6,
            StopType::Seconds => 4,
        }
    }
}

/// Error flags returned by `error_flags()`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ErrorFlags {
    /// No error.
    None,
    /// RPM deviation exceeded threshold.
    SpeedError,
    /// Driver fault (overcurrent, overtemperature, undervoltage).
    DriverError,
}

impl fmt::Display for ErrorFlags {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ErrorFlags::None => write!(f, "no error"),
            ErrorFlags::SpeedError => write!(f, "speed deviation error"),
            ErrorFlags::DriverError => write!(f, "driver error"),
        }
    }
}
