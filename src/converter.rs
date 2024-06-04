pub mod to_nav;
pub mod from_nav;



use crate::runtime::{RtResult};



/// The trait is a simple and generic converter interface. Predominantly is used to handle runtime tree.
pub trait Converter {
    type Output;

    fn convert(&self) -> RtResult<Self::Output>;
}
