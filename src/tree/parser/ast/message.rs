use crate::tree::parser::ast::arg::MesType;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt::{Display, Formatter};

#[derive(Debug, Copy, Clone, PartialEq, Deserialize, Serialize)]
pub enum Number {
    Int(i64),
    Float(f64),
    Hex(i64),
    Binary(isize),
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct StringLit(pub String);

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
pub enum Bool {
    True,
    False,
}

impl Into<bool> for Bool {
    fn into(self) -> bool {
        match self {
            Bool::True => true,
            Bool::False => false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
pub enum Message {
    Num(Number),
    String(StringLit),
    Bool(Bool),
    Array(Vec<Message>),
    Object(HashMap<String, Message>),
}

impl Display for Message {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Message::Num(v) => match v {
                Number::Int(v) => write!(f, "{}", v),
                Number::Float(v) => write!(f, "{}", v),
                Number::Hex(v) => write!(f, "{}", v),
                Number::Binary(v) => write!(f, "{}", v),
            },
            Message::String(v) => write!(f, "{}", v.0),
            Message::Bool(b) => match b {
                Bool::True => write!(f, "true"),
                Bool::False => write!(f, "false"),
            },
            Message::Array(array) => {
                let mut list = f.debug_list();
                let strings: Vec<_> = array.iter().map(|v| format!("{}", v)).collect();
                list.entries(strings);
                list.finish()
            }
            Message::Object(obj) => {
                let mut map = f.debug_map();
                let entries: Vec<_> = obj.iter().map(|(k, v)| (k, format!("{}", v))).collect();
                map.entries(entries);
                map.finish()
            }
        }
    }
}

impl Message {
    pub fn same(&self, mt: &MesType) -> bool {
        match (&self, mt) {
            (Message::Num(_), MesType::Num) => true,
            (Message::String(_), MesType::String) => true,
            (Message::Bool(_), MesType::Bool) => true,
            (Message::Array(_), MesType::Array) => true,
            (Message::Object(_), MesType::Object) => true,
            _ => false,
        }
    }

    pub fn str(v: &str) -> Self {
        Message::String(StringLit(v.to_string()))
    }
    pub fn bool(v: bool) -> Self {
        if v {
            Message::Bool(Bool::True)
        } else {
            Message::Bool(Bool::False)
        }
    }
    pub fn int(v: i64) -> Self {
        Message::Num(Number::Int(v))
    }
    pub fn float(v: f64) -> Self {
        Message::Num(Number::Float(v))
    }
    pub fn object(pairs: Vec<(String, Message)>) -> Self {
        Message::Object(HashMap::from_iter(pairs))
    }
    pub fn array(elems: Vec<Message>) -> Self {
        Message::Array(elems)
    }
}
