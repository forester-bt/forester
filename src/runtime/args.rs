use std::collections::HashMap;
use crate::runtime::blackboard::BBKey;

pub type RtArgKey = String;

pub enum RtValueNumber {
    Int(i64),
    Float(f64),
    Hex(i64),
    Binary(isize)
}

pub enum RtValue {
    String(String),
    Bool(bool),
    Array(Vec<RtValue>),
    Object(HashMap<String, RtValue>),
    Number(RtValueNumber),
    Pointer(BBKey)
}

pub struct RtArgs(Vec<RtArgument>);

pub struct RtArgument {
    name: RtArgKey,
    value: RtValue,
}