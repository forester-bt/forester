pub mod display;
pub mod transform;

use crate::runtime::blackboard::{BBKey, BlackBoard};
use crate::runtime::rtree::rnode::DecoratorType;
use crate::runtime::{RtResult, RuntimeError};
use crate::tree::parser::ast::arg::{
    Argument, ArgumentRhs, Arguments, ArgumentsType, MesType, Param, Params,
};
use crate::tree::parser::ast::call::Call;
use crate::tree::parser::ast::message::{Message, Number};
use crate::tree::parser::ast::Key;
use crate::tree::{cerr, TreeError};
use itertools::Itertools;
use std::collections::HashMap;
use std::fmt::{Display, Formatter, Write};

pub type RtAKey = String;

#[derive(Debug, PartialEq, Clone)]
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
#[derive(Debug, PartialEq, Clone)]
pub enum RtValue {
    String(String),
    Bool(bool),
    Array(Vec<RtValue>),
    Object(HashMap<String, RtValue>),
    Number(RtValueNumber),
    Pointer(BBKey),
    Call(Call),
}

pub struct RtValueCast<'a> {
    v: RtValue,
    bb: &'a BlackBoard,
}

impl<'a> RtValueCast<'a> {
    fn chain(self) -> RtResult<RtValue> {
        self.v.chain(self.bb)
    }

    pub fn string(self) -> RtResult<Option<String>> {
        self.chain().map(RtValue::as_string)
    }
    pub fn int(self) -> RtResult<Option<i64>> {
        self.chain().map(RtValue::as_int)
    }
    pub fn bool(self) -> RtResult<Option<bool>> {
        self.chain().map(RtValue::as_bool)
    }
    pub fn float(self) -> RtResult<Option<f64>> {
        self.chain().map(RtValue::as_float)
    }
    pub fn vec<Map, To>(self, map: Map) -> RtResult<Option<Vec<To>>>
    where
        Map: Fn(RtValue) -> To,
    {
        self.chain().map(|v| v.as_vec(map))
    }
    pub fn map<Map, To>(self, map: Map) -> RtResult<Option<HashMap<String, To>>>
    where
        Map: Fn((String, RtValue)) -> (String, To),
    {
        self.chain().map(|v| v.as_map(map))
    }
}

impl RtValue {
    pub fn int(i: i64) -> Self {
        RtValue::Number(RtValueNumber::Int(i))
    }
    pub fn str(s: String) -> Self {
        RtValue::String(s)
    }

    pub fn cast(self, bb: &BlackBoard) -> RtValueCast {
        RtValueCast { v: self, bb }
    }

    pub fn as_string(self) -> Option<String> {
        match self {
            RtValue::String(v) => Some(v),
            _ => None,
        }
    }
    pub fn as_bool(self) -> Option<bool> {
        match self {
            RtValue::Bool(v) => Some(v),
            _ => None,
        }
    }
    pub fn as_vec<Map, To>(self, map: Map) -> Option<Vec<To>>
    where
        Map: Fn(RtValue) -> To,
    {
        match self {
            RtValue::Array(elems) => Some(elems.into_iter().map(|v| map(v)).collect()),
            _ => None,
        }
    }
    pub fn as_map<Map, To>(self, map: Map) -> Option<HashMap<String, To>>
    where
        Map: Fn((String, RtValue)) -> (String, To),
    {
        match self {
            RtValue::Object(elems) => Some(HashMap::from_iter(
                elems.into_iter().map(|v| map(v)).collect::<Vec<_>>(),
            )),
            _ => None,
        }
    }
    pub fn as_int(self) -> Option<i64> {
        match self {
            RtValue::Number(RtValueNumber::Int(i)) => Some(i),
            _ => None,
        }
    }
    pub fn as_float(self) -> Option<f64> {
        match self {
            RtValue::Number(RtValueNumber::Float(f)) => Some(f),
            _ => None,
        }
    }

    pub fn as_pointer(self) -> Option<String> {
        match self {
            RtValue::Pointer(k) => Some(k),
            _ => None,
        }
    }

    pub fn chain(self, bb: &BlackBoard) -> RtResult<RtValue> {
        match self {
            RtValue::Pointer(p) => {
                let mut value = bb.get(p)?.expect("something");
                while let Some(p) = value.clone().as_pointer() {
                    value = bb.get(p)?.expect("something");
                    debug!(target:"  chain","{value}");
                }
                Ok(value.clone())
            }
            v => Ok(v),
        }
    }
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

#[derive(Default, Debug, PartialEq, Clone)]
pub struct RtArgs(pub Vec<RtArgument>);

impl RtArgs {
    pub fn first(&self) -> Option<RtValue> {
        self.0.first().map(|a| a.value.clone())
    }
    pub fn first_as<M, To>(&self, map: M) -> Option<To>
    where
        M: Fn(RtValue) -> Option<To>,
    {
        self.0.first().and_then(|v| map(v.value.clone()))
    }
    pub fn find(&self, key: RtAKey) -> Option<RtValue> {
        self.0
            .iter()
            .find(|a| a.name == key)
            .map(|a| a.clone().value)
    }
    pub fn find_or_ith(&self, key: RtAKey, ith: usize) -> Option<RtValue> {
        self.0
            .iter()
            .find(|a| a.name == key)
            .or(self.0.get(ith))
            .map(|a| a.clone().value)
    }

    pub fn with(self, key: &str, value: RtValue) -> RtArgs {
        let mut elems = self.0;
        let cursor = elems.iter().position(|e| e.clone().name() == key);
        let new_elem = RtArgument::new(key.to_string(), value);
        match cursor {
            None => {
                elems.push(new_elem);
            }
            Some(idx) => {
                elems.remove(idx);
                elems.insert(idx, new_elem)
            }
        }

        RtArgs(elems)
    }
}

impl Display for RtArgs {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let elems = self.0.iter().map(|v| v.to_string()).join(",");
        f.write_str(elems.as_str())?;
        Ok(())
    }
}

#[derive(Debug, PartialEq, Clone)]
pub struct RtArgument {
    name: RtAKey,
    value: RtValue,
}

impl Display for RtValueNumber {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            RtValueNumber::Int(v) => f.write_str(format!("{}", v).as_str()),
            RtValueNumber::Float(v) => f.write_str(format!("{}", v).as_str()),
            RtValueNumber::Hex(v) => f.write_str(format!("0x{:02x}", v).as_str()),
            RtValueNumber::Binary(v) => f.write_str(format!("{:#b}", v).as_str()),
        }
    }
}
impl Display for RtValue {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            RtValue::String(v) => f.write_str(v)?,
            RtValue::Bool(b) => f.write_str(format!("{}", b).as_str())?,
            RtValue::Array(elems) => {
                let elems = elems.iter().map(|e| e.to_string()).join(",");
                f.write_str(format!("[{}]", elems).as_str())?;
            }
            RtValue::Object(obj) => {
                let elems = obj.iter().map(|(k, v)| format!("{}:{}", k, v)).join(",");
                f.write_str(format!("[{}]", elems).as_str())?;
            }
            RtValue::Number(n) => f.write_str(format!("{}", n).as_str())?,
            RtValue::Pointer(p) => f.write_str(format!("{}", p).as_str())?,
            RtValue::Call(_) => f.write_str(format!("<Call>>").as_str())?,
        }
        Ok(())
    }
}

impl Display for RtArgument {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.write_str(format!("{}={}", &self.name, &self.value).as_str())?;
        Ok(())
    }
}

impl RtArgument {
    pub fn val(self) -> RtValue {
        self.value
    }
    pub fn name(self) -> RtAKey {
        self.name
    }
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
