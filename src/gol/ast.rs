use std::collections::HashMap;
use std::fmt::{Formatter};
use std::str::FromStr;
use parsit::step::Step;
use strum::ParseError;
use strum_macros::EnumString;


#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Number {
    Int(i64),
    Float(f64),
    Hex(i64),
    Binary(isize),
}

#[derive(Debug, Clone, PartialEq, Hash)]
pub struct Id(pub String);

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

impl Message {
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
    Invocation(Id, Arguments),
    Lambda(TreeType, Calls),
    Decorator(TreeType, Arguments, Box<Call>),
}

impl Call {
    pub fn invocation(id: &str, args: Arguments) -> Self {
        Call::Invocation(Id(id.to_string()), args)
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
    pub name: Id,
    pub tpe: MesType,
}

impl Param {
    pub fn new(id: &str, tpe: MesType) -> Self {
        Param { name: Id(id.to_string()), tpe }
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
    Id(Id),
    Mes(Message),
    AssignedId(Id, Id),
    AssignedMes(Id, Message),
}

impl Argument {
    pub fn id(v: &str) -> Self {
        Argument::Id(Id(v.to_string()))
    }
    pub fn mes(v: Message) -> Self {
        Argument::Mes(v)
    }
    pub fn id_id(lhs: &str, rhs: &str) -> Self {
        Argument::AssignedId(Id(lhs.to_string()), Id(rhs.to_string()))
    }
    pub fn id_mes(lhs: &str, rhs: Message) -> Self {
        Argument::AssignedMes(Id(lhs.to_string()), rhs)
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

#[derive(Debug, Clone, Eq, PartialEq, EnumString)]
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
    pub name: Id,
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
    pub fn new(tpe: TreeType, name: Id, params: Params, calls: Calls) -> Self {
        Self { tpe, name, params, calls }
    }
}

#[derive(Clone, Debug,PartialEq)]
pub enum Import {
    File(String),
    Names(String, Vec<ImportName>),
}
#[derive(Clone, Debug,PartialEq)]
pub enum ImportName{
    Id(String),
    Alias(String,String)
}

impl ImportName {
    pub fn id(v:&str) -> Self {
        ImportName::Id(v.to_string())
    }
    pub fn alias(v:&str,alias:&str) -> Self {
        ImportName::Alias(v.to_string(),alias.to_string())
    }
}


impl Import {
    pub fn name(&self) -> &str {
        match self {
            Import::File(n) => n,
            Import::Names(n, _) => n,
        }
    }
    pub fn file(f: &str) -> Self {
        Import::File(f.to_string())
    }
    pub fn names(f: &str, names: Vec<&str>) -> Self {
        Import::Names(
            f.to_string(),
            names.into_iter().map(|v| ImportName::id(v)).collect(),
        )
    }
    pub fn names_mixed(f: &str, names: Vec<ImportName>) -> Self {
        Import::Names(
            f.to_string(),
            names,
        )
    }
}

#[derive(Clone, Debug,PartialEq)]
pub enum FileEntity {
    Tree(Tree),
    Import(Import),
}

#[derive(Clone, Debug,PartialEq)]
pub struct AstFile(pub Vec<FileEntity>);

impl<'a> AstFile {
    pub fn new(field0: Vec<FileEntity>) -> Self {
        Self(field0)
    }
}


#[cfg(test)]
mod tests {
    use std::str::FromStr;
    use crate::gol::ast::TreeType;

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