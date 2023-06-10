use std::collections::HashMap;
use std::fmt::{Display, Formatter, Write};
use std::str::FromStr;
use parsit::step::Step;
use strum::ParseError;
use strum_macros::EnumString;
use strum_macros::Display;
use crate::tree::project::{AliasName, TreeName};
use crate::tree::project::invocation::Invocation;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Number {
    Int(i64),
    Float(f64),
    Hex(i64),
    Binary(isize),
}


pub type Key = String;

#[derive(Clone, Debug, PartialEq)]
pub struct StringLit(pub String);

#[derive(Debug, Clone, PartialEq)]
pub enum Bool {
    True,
    False,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Message {
    Num(Number),
    String(StringLit),
    Bool(Bool),
    Array(Vec<Message>),
    Object(HashMap<String, Message>),
    Call(Call),
}

impl Display for Message {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Message::Num(v) => {match v {
                Number::Int(v) => f.write_str(v.to_string().as_str()),
                Number::Float(v) => f.write_str(v.to_string().as_str()),
                Number::Hex(v) => f.write_str(v.to_string().as_str()),
                Number::Binary(v) => f.write_str(v.to_string().as_str()),
            }}
            Message::String(v) => f.write_str(v.0.as_str()),
            Message::Bool(b) => match b {
                Bool::True => f.write_str("true"),
                Bool::False => f.write_str("false"),
            },
            Message::Array(a) => {
            }
            Message::Object(_) => {}
            Message::Call(_) => {}
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
            (Message::Call(_), MesType::Tree) => true,
            _ => false
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
    pub fn invocation(id: &str, args: Arguments) -> Self {
        Message::Call(Call::invocation(id, args))
    }

    pub fn lambda(tpe: TreeType, calls: Calls) -> Self {
        Message::Call(Call::lambda(tpe, calls))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Call {
    Invocation(Key, Arguments),
    Lambda(TreeType, Calls),
    Decorator(TreeType, Arguments, Box<Call>),
}

impl Call {
    pub fn invocation(id: &str, args: Arguments) -> Self {
        Call::Invocation(id.to_string(), args)
    }
    pub fn lambda(tpe: TreeType, calls: Calls) -> Self {
        Call::Lambda(tpe, calls)
    }
    pub fn decorator(tpe: TreeType, args: Arguments, call: Call) -> Self {
        Call::Decorator(tpe, args, Box::new(call))
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Calls {
    pub elems: Vec<Call>,
}

impl Calls {
    pub fn new(elems: Vec<Call>) -> Self {
        Calls { elems }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Param {
    pub name: Key,
    pub tpe: MesType,
}

impl Param {
    pub fn new(id: &str, tpe: MesType) -> Self {
        Param { name: id.to_string(), tpe }
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

#[derive(Clone, Debug, PartialEq)]
pub enum Argument {
    Id(Key),
    Mes(Message),
    AssignedId(Key, Key),
    AssignedMes(Key, Message),
}

impl Argument {
    pub fn name(&self) -> Option<&Key> {
        match self {
            Argument::Id(_) | Argument::Mes(_) => None,
            Argument::AssignedId(k, _) | Argument::AssignedMes(k, _) => Some(k)
        }
    }

    pub fn id(v: &str) -> Self {
        Argument::Id(v.to_string())
    }
    pub fn mes(v: Message) -> Self {
        Argument::Mes(v)
    }
    pub fn id_id(lhs: &str, rhs: &str) -> Self {
        Argument::AssignedId(lhs.to_string(), rhs.to_string())
    }
    pub fn id_mes(lhs: &str, rhs: Message) -> Self {
        Argument::AssignedMes(lhs.to_string(), rhs)
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Arguments {
    pub args: Vec<Argument>,
}

impl<'a> Arguments {
    pub fn new(args: Vec<Argument>) -> Self {
        Self { args }
    }
}

#[derive(Display, Debug, Clone, Eq, PartialEq, EnumString)]
#[strum(serialize_all = "snake_case")]
pub enum TreeType {
    Root,
    Parallel,
    Sequence,
    MSequence,
    RSequence,
    Fallback,
    RFallback,
    // decorators
    Inverter,
    ForceSuccess,
    ForceFail,
    Repeat,
    Retry,
    Timeout,
    // actions
    Impl,
    Cond,
}

impl TreeType {
    pub fn is_decorator(&self) -> bool {
        match self {
            TreeType::Inverter |
            TreeType::ForceSuccess |
            TreeType::ForceFail |
            TreeType::Repeat |
            TreeType::Retry |
            TreeType::Timeout => true,
            _ => false
        }
    }
}

pub fn validate_lambda<'a, 'b>(tpe: &'a TreeType, args: &'a Arguments, calls: &'a Calls) -> Result<(), &'b str> {
    match tpe {
        TreeType::Impl | TreeType::Cond => Err("the types impl or cond should have declaration and get called by name"),

        _ if tpe.is_decorator() => {
            if calls.elems.len() != 1 {
                Err("any decorator should have only one child")
            } else {
                Ok(())
            }
        }

        _ => {
            if args.args.is_empty() {
                Ok(())
            } else {
                Err("any lambda invocation should not have arguments")
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum MesType {
    Num,
    Array,
    Object,
    String,
    Bool,
    Tree,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Tree {
    pub tpe: TreeType,
    pub name: Key,
    pub params: Params,
    pub calls: Calls,
}

impl Tree {
    pub fn is_root(&self) -> bool {
        match self.tpe {
            TreeType::Root => true,
            _ => false
        }
    }
    pub fn new(tpe: TreeType, name: Key, params: Params, calls: Calls) -> Self {
        Self { tpe, name, params, calls }
    }
    pub fn to_inv(&self) -> Invocation {
        self.into()
    }
    pub fn to_inv_args(&self, args: Arguments) -> Invocation {
        Invocation::new(self, args)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Import(pub String, pub Vec<ImportName>);

#[derive(Clone, Debug, PartialEq, Hash, Eq)]
pub enum ImportName {
    Id(String),
    Alias(TreeName, AliasName),
    WholeFile,
}

impl ImportName {
    pub fn id(v: &str) -> Self {
        ImportName::Id(v.to_string())
    }
    pub fn alias(v: &str, alias: &str) -> Self {
        ImportName::Alias(v.to_string(), alias.to_string())
    }
}


impl Import {
    pub fn f_name(&self) -> &str {
        match self {
            Import(n, _) => n,
        }
    }
    pub fn file(f: &str) -> Self {
        Import(f.to_string(), vec![ImportName::WholeFile])
    }
    pub fn names(f: &str, names: Vec<&str>) -> Self {
        Import(
            f.to_string(),
            names.into_iter().map(|v| ImportName::id(v)).collect(),
        )
    }
    pub fn names_mixed(f: &str, names: Vec<ImportName>) -> Self {
        Import(
            f.to_string(),
            names,
        )
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum FileEntity {
    Tree(Tree),
    Import(Import),
}

#[derive(Clone, Debug, PartialEq)]
pub struct AstFile(pub Vec<FileEntity>);

impl<'a> AstFile {
    pub fn new(field0: Vec<FileEntity>) -> Self {
        Self(field0)
    }
}


#[cfg(test)]
mod tests {
    use std::str::FromStr;
    use crate::tree::parser::ast::TreeType;

    #[test]
    fn enum_test() {
        assert_eq!(
            TreeType::from_str("cond").unwrap(),
            TreeType::Cond
        );
        assert!(
            TreeType::from_str("bla").is_err()
        )
    }
}