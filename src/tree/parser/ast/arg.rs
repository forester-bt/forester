use crate::tree::parser::ast::arg::ArgumentsType::{Named, Unnamed};
use crate::tree::parser::ast::call::Call;
use crate::tree::parser::ast::message::Message;
use crate::tree::parser::ast::Key;
use crate::tree::{cerr, TreeError};
use itertools::Itertools;
use serde::{Deserialize, Serialize};
use std::fmt::{Display, Formatter, Write};

/// Just a pair of name and type
/// This is a representation of a tree parameter
/// # Example
/// `sequence tree(a:string){...}`
/// and a:string is a param
#[derive(Clone, Debug, PartialEq)]
pub struct Param {
    pub name: Key,
    pub tpe: MesType,
}

impl Display for Param {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}:{}", self.name, self.tpe)
    }
}

impl Param {
    pub fn new(id: &str, tpe: MesType) -> Self {
        Param {
            name: id.to_string(),
            tpe,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Params {
    pub params: Vec<Param>,
}

impl Params {
    pub fn new(params: Vec<Param>) -> Self {
        Params { params }
    }
}

/// The right part of the argument
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub enum ArgumentRhs {
    /// A pointer to the bb key or the parameter from the parent tree
    Id(Key),
    /// A scalar value
    Mes(Message),
    /// A call to a tree
    Call(Call),
}

impl ArgumentRhs {
    pub fn get_call(&self) -> Option<Call> {
        match self {
            ArgumentRhs::Call(call) => Some(call.clone()),
            _ => None,
        }
    }
}

impl Display for ArgumentRhs {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            ArgumentRhs::Id(id) => write!(f, "&{}", id),
            ArgumentRhs::Mes(m) => write!(f, "{}", m),
            ArgumentRhs::Call(c) => match c {
                Call::Invocation(name, args) => {
                    write!(f, "{}({})", name, args)
                }
                Call::HoInvocation(name) => {
                    write!(f, "{}(..)", name)
                }
                Call::Lambda(tpe, _) => {
                    write!(f, "{}...", tpe)
                }
                Call::Decorator(tpe, args, _call) => {
                    write!(f, "{}({})...", tpe, args)
                }
            },
        }
    }
}

/// An argument is a pair of name and value or just a value
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub enum Argument {
    Assigned(Key, ArgumentRhs),
    Unassigned(ArgumentRhs),
}

impl Argument {
    pub fn has_name(&self, key: &Key) -> bool {
        matches!(self, Argument::Assigned(k, _) if k == key)
    }

    pub fn name(&self) -> Option<&Key> {
        match self {
            Argument::Assigned(k, _) => Some(k),
            Argument::Unassigned(_) => None,
        }
    }

    pub fn value(&self) -> &ArgumentRhs {
        match self {
            Argument::Assigned(_, v) | Argument::Unassigned(v) => v,
        }
    }
}

impl Display for Argument {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Argument::Assigned(k, rhs) => write!(f, "{}={}", k, rhs),
            Argument::Unassigned(rhs) => write!(f, "{}", rhs),
        }
    }
}

impl Argument {
    pub fn id(v: &str) -> Self {
        Argument::Unassigned(ArgumentRhs::Id(v.to_string()))
    }
    pub fn mes(v: Message) -> Self {
        Argument::Unassigned(ArgumentRhs::Mes(v))
    }
    pub fn call(v: Call) -> Self {
        Argument::Unassigned(ArgumentRhs::Call(v))
    }
    pub fn id_id(lhs: &str, rhs: &str) -> Self {
        Argument::Assigned(lhs.to_string(), ArgumentRhs::Id(rhs.to_string()))
    }
    pub fn id_mes(lhs: &str, rhs: Message) -> Self {
        Argument::Assigned(lhs.to_string(), ArgumentRhs::Mes(rhs))
    }
    pub fn id_call(lhs: &str, rhs: Call) -> Self {
        Argument::Assigned(lhs.to_string(), ArgumentRhs::Call(rhs))
    }
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
pub struct Arguments {
    pub args: Vec<Argument>,
}

/// this is a plain representation of the argument type
/// to avoid several levels of nesting
pub enum ArgumentsType {
    Unnamed,
    Named,
    Empty,
}

impl Arguments {

    pub fn get_type(&self) -> Result<ArgumentsType, TreeError> {
        let mut curr = None;

        for a in &self.args {
            match (a, &curr) {
                (Argument::Assigned(_, _), None) => curr = Some(Named),
                (Argument::Unassigned(_), None) => curr = Some(Unnamed),
                (Argument::Assigned(_, _), Some(Named)) => {}
                (Argument::Unassigned(_), Some(Unnamed)) => {}
                _ => {
                    return Err(cerr(
                        format!("the arguments ({}) should be either named ot unnamed but not a mix",self)
                    ))
                }
            }
        }
        Ok(curr.unwrap_or(ArgumentsType::Empty))
    }
}

impl Display for Arguments {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let str = &self.args.iter().map(|a| format!("{}", a)).join(",");
        write!(f, "{}", str)
    }
}

impl Arguments {
    pub fn new(args: Vec<Argument>) -> Self {
        Self { args }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum MesType {
    Any,
    Num,
    Array,
    Object,
    String,
    Bool,
    Tree,
}

impl Display for MesType {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            MesType::Any => write!(f, "any"),
            MesType::Num => write!(f, "num"),
            MesType::Array => write!(f, "array"),
            MesType::Object => write!(f, "object"),
            MesType::String => write!(f, "string"),
            MesType::Bool => write!(f, "bool"),
            MesType::Tree => write!(f, "tree"),
        }
    }
}
