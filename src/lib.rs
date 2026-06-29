//! Rust driver for iarduino I2C motor controllers (FLASH-I2C series).
//!
//! Provides both synchronous (via [`embedded_hal::i2c::I2c`]) and optional
//! asynchronous (via [`MotorAsyncExt`], behind `feature = "async"`) operation.
//!
//! # Cargo features
//!
//! * `async` — enables async API via the [`MotorAsyncExt`] trait (requires
//!   `embedded-io-async`).
//!
//! # Minimum Supported Rust Version (MSRV)
//!
//! Rust 1.81 (required by `embedded-io` 0.7).

#![no_std]

mod error;
mod motor;
mod registers;

pub use error::MotorError;
pub use motor::Motor;
pub use registers::{
    ErrorFlags, SpeedType, StopType,
    DEF_CHIP_ID_FLASH, DEF_CHIP_ID_METRO, DEF_MODEL_MOT,
    MOT_ERR_DRV, MOT_ERR_SPD,
};

#[cfg(feature = "async")]
pub use motor::{AsyncI2c, MotorAsyncExt};
