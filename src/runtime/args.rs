pub mod display;
pub mod transform;

use crate::runtime::blackboard::BBKey;
use crate::runtime::rnode::DecoratorType;
use crate::tree::parser::ast::arg::{
    Argument, ArgumentRhs, Arguments, ArgumentsType, MesType, Param, Params,
};
use crate::tree::parser::ast::call::Call;
use crate::tree::parser::ast::message::{Message, Number};
use crate::tree::parser::ast::Key;
use crate::tree::{cerr, TreeError};
use itertools::Itertools;
use std::collections::HashMap;
use std::fmt::{Display, Formatter};

pub type RtAKey = String;
#[derive(Debug, PartialEq)]
pub enum RtValueNumber {
    Int(i64),
    Float(f64),
    Hex(i64),
    Binary(isize),
}

impl From<Number> for RtValueNumber {
    fn from(value: Number) -> Self {
        match value {
            Number::Int(i) => RtValueNumber::Int(i),
            Number::Float(f) => RtValueNumber::Float(f),
            Number::Hex(h) => RtValueNumber::Hex(h),
            Number::Binary(b) => RtValueNumber::Binary(b),
        }
    }
}
#[derive(Debug, PartialEq)]
pub enum RtValue {
    String(String),
    Bool(bool),
    Array(Vec<RtValue>),
    Object(HashMap<String, RtValue>),
    Number(RtValueNumber),
    Pointer(BBKey),
    Call(Call),
}

impl From<Message> for RtValue {
    fn from(value: Message) -> Self {
        match value {
            Message::Num(n) => RtValue::Number(n.into()),
            Message::String(s) => RtValue::String(s.0),
            Message::Bool(b) => RtValue::Bool(b.into()),
            Message::Array(elems) => RtValue::Array(elems.into_iter().map(Into::into).collect()),
            Message::Object(elems) => {
                RtValue::Object(elems.into_iter().map(|(k, v)| (k, v.into())).collect())
            }
        }
    }
}

#[derive(Default, Debug, PartialEq)]
pub struct RtArgs(Vec<RtArgument>);
#[derive(Debug, PartialEq)]
pub struct RtArgument {
    name: RtAKey,
    value: RtValue,
}

impl RtArgument {
    pub fn new(name: RtAKey, value: RtValue) -> Self {
        Self { name, value }
    }
    pub fn new_noname(value: RtValue) -> Self {
        Self {
            name: "arg".to_string(),
            value,
        }
    }

    pub fn try_from(a: ArgumentRhs, p: Param) -> Result<Option<RtArgument>, TreeError> {
        let _ = RtArgument::validate_type(a.clone(), p.tpe)?;
        match &a {
            ArgumentRhs::Id(id) => Ok(Some(RtArgument::new(p.name, RtValue::Pointer(id.clone())))),
            ArgumentRhs::Mes(m) => Ok(Some(RtArgument::new(p.name, m.clone().into()))),
            ArgumentRhs::Call(c) => Ok(Some(RtArgument::new(p.name, RtValue::Call(c.clone())))),
        }
    }
    pub fn validate_type(arg: ArgumentRhs, param: MesType) -> Result<(), TreeError> {
        let error = |lhs: &str, rhs: &str| {
            Err(cerr(format!(
                "the type of argument {lhs} does not coincide to the type of the definition {rhs}",
            )))
        };

        match (arg, param) {
            (ArgumentRhs::Call(_), MesType::Tree) => Ok(()),
            (ArgumentRhs::Id(_), MesType::Tree) => error("pointer", "call"),
            (ArgumentRhs::Mes(_), MesType::Tree) => error("message", "call"),

            (ArgumentRhs::Call(_), m) => error("call", format!("{:?}", m).as_str()),
            (ArgumentRhs::Id(_), _) => Ok(()),

            (ArgumentRhs::Mes(m), m_t) if m.same(&m_t) => Ok(()),
            (ArgumentRhs::Mes(m), m_t) => {
                error(format!("{}", m).as_str(), format!("{:?}", m_t).as_str())
            }
        }
    }
}
