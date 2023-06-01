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

#[derive(Debug, Copy, Clone, PartialEq, Hash)]
pub struct Id<'a>(pub &'a str);

#[derive(Clone, Debug,PartialEq)]
pub struct StringLit<'a>(pub &'a str);

#[derive(Debug, Clone,PartialEq)]
pub enum Bool {
    True,
    False,
}

#[derive(Debug, Clone,PartialEq)]
pub enum Message<'a> {
    Num(Number),
    String(StringLit<'a>),
    Bool(Bool),
    Array(Vec<Message<'a>>),
    Object(HashMap<String, Message<'a>>),
    Call(Call<'a>),
}

#[derive(Debug, Clone,PartialEq)]
pub enum Call<'a> {
    Invocation(Id<'a>, Arguments<'a>),
    Lambda(TreeType, Option<Id<'a>>, Calls<'a>),
}

#[derive(Clone, Debug, Default,PartialEq)]
pub struct Calls<'a> {
    pub elems: Vec<Call<'a>>,
}

#[derive(Clone, Debug,PartialEq)]
pub struct Param<'a> {
    pub name: Id<'a>,
    pub tpe: MesType,
}

#[derive(Clone, Debug, Default,PartialEq)]
pub struct Params<'a> {
    pub params: Vec<Param<'a>>,
}

#[derive(Clone, Debug,PartialEq)]
pub enum Argument<'a> {
    Id(Id<'a>),
    Mes(Message<'a>),
    AssignedId(Id<'a>, Id<'a>),
    AssignedMes(Id<'a>, Message<'a>),
}

#[derive(Clone, Debug, Default,PartialEq)]
pub struct Arguments<'a> {
    pub args: Vec<Argument<'a>>,
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
    Wait,
    Success,
    Fail,
    // conditions
    Cond,
    Equal,
    Greater,
    Less,
    Test,
    TestAll,
    TestOne,
}


#[derive(Clone, Debug,PartialEq)]
pub enum MesType {
    Num,
    Array,
    Object,
    String,
    Bool,
    Tree,
}

#[derive(Clone, Debug,PartialEq)]
pub struct Tree<'a> {
    pub tpe: TreeType,
    pub name: Id<'a>,
    pub params: Params<'a>,
    pub calls: Calls<'a>,
}

#[derive(Clone, Debug)]
pub struct Trees<'a>(pub HashMap<String, Tree<'a>>);

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