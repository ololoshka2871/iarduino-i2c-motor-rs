//! Motor driver error type implementing [`embedded_io::Error`].

use core::fmt;
use embedded_io;

/// Errors that can occur during motor driver operations.
#[derive(Debug)]
pub enum MotorError {
    /// I2C bus communication failure.
    I2c,
    /// Device not initialized (call [`begin`](crate::Motor::begin) first).
    NotInitialized,
    /// Invalid parameter value.
    InvalidParam,
    /// Operation not supported by this hardware revision.
    Unsupported,
    /// I2C transaction timed out.
    Timeout,
}

impl fmt::Display for MotorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MotorError::I2c => write!(f, "I2C communication error"),
            MotorError::NotInitialized => write!(f, "motor not initialized"),
            MotorError::InvalidParam => write!(f, "invalid parameter"),
            MotorError::Unsupported => write!(f, "operation not supported by hardware"),
            MotorError::Timeout => write!(f, "I2C timeout"),
        }
    }
}

impl core::error::Error for MotorError {}

impl embedded_io::Error for MotorError {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            MotorError::I2c => embedded_io::ErrorKind::ConnectionAborted,
            MotorError::NotInitialized => embedded_io::ErrorKind::NotConnected,
            MotorError::InvalidParam => embedded_io::ErrorKind::InvalidInput,
            MotorError::Unsupported => embedded_io::ErrorKind::Unsupported,
            MotorError::Timeout => embedded_io::ErrorKind::TimedOut,
        }
    }
}
