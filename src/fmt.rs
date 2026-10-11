//! Logging macros that go to `defmt` when the feature is on, and nowhere otherwise.
#![allow(unused_macros)]

macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        #[cfg(feature = "defmt")]
        ::defmt::trace!($s $(, $x)*);
        #[cfg(not(feature = "defmt"))]
        let _ = ($( & $x ),*);
    }};
}

macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        #[cfg(feature = "defmt")]
        ::defmt::info!($s $(, $x)*);
        #[cfg(not(feature = "defmt"))]
        let _ = ($( & $x ),*);
    }};
}

macro_rules! warning {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        #[cfg(feature = "defmt")]
        ::defmt::warn!($s $(, $x)*);
        #[cfg(not(feature = "defmt"))]
        let _ = ($( & $x ),*);
    }};
}

macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        #[cfg(feature = "defmt")]
        ::defmt::debug!($s $(, $x)*);
        #[cfg(not(feature = "defmt"))]
        let _ = ($( & $x ),*);
    }};
}

#[allow(unused_imports)]
pub(crate) use {debug, info, trace, warning};
