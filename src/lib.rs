//! This is a platform-agnostic Rust driver for the TMP108 temperature sensor
//! based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! For further details of the device architecture and operation, please refer
//! to the official [`Datasheet`].
//!
//! [`Datasheet`]: https://www.ti.com/lit/gpn/tmp108

#![doc(html_root_url = "https://docs.rs/tmp108/latest")]
#![doc = include_str!("../README.md")]
#![cfg_attr(not(test), no_std)]

mod registers;
pub use registers::*;

#[cfg(feature = "async")]
pub mod asynchronous;

pub mod blocking;

/// A0 pin logic level representation.
#[derive(Debug)]
pub enum A0 {
    /// A0 tied to GND (default).
    Gnd,
    /// A0 tied to V+.
    Vplus,
    /// A0 tied to SDA.
    Sda,
    /// A0 tied to SCL.
    Scl,
}

impl Default for A0 {
    fn default() -> Self {
        Self::Gnd
    }
}

impl From<A0> for u8 {
    fn from(connection: A0) -> Self {
        match connection {
            A0::Gnd => 0b100_1000,
            A0::Vplus => 0b100_1001,
            A0::Sda => 0b100_1010,
            A0::Scl => 0b100_1011,
        }
    }
}
