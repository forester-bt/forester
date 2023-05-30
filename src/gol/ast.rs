use std::fmt::{Display, Formatter};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct EmptyToken {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Number {
    Int(i64),
    Float(f64),
    Hex(i64),
    Binary(isize),
}
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct  Id<'a>(pub &'a str);



pub struct Definition{}
pub struct Definitions(Vec<Definition>);