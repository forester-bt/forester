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

#[derive(Clone, Debug, PartialEq)]
pub struct StringLit<'a>(pub &'a str);

#[derive(Debug, Clone, PartialEq)]
pub enum Bool {
    True,
    False,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Message<'a> {
    Num(Number),
    String(StringLit<'a>),
    Bool(Bool),
    Array(Vec<Message<'a>>),
    Object(HashMap<String, Message<'a>>),
    Call(Call<'a>),
}

impl<'a> Message<'a> {
    pub fn str(v: &'a str) -> Self {
        Message::String(StringLit(v))
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
    pub fn object(pairs: Vec<(String, Message<'a>)>) -> Self {
        Message::Object(HashMap::from_iter(pairs))
    }
    pub fn array(elems: Vec<Message<'a>>) -> Self {
        Message::Array(elems)
    }
    pub fn invocation(id: &'a str, args: Arguments<'a>) -> Self {
        Message::Call(Call::invocation(id, args))
    }

    pub fn lambda(tpe: TreeType, calls: Calls<'a>) -> Self {
        Message::Call(Call::lambda(tpe, calls))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Call<'a> {
    Invocation(Id<'a>, Arguments<'a>),
    Lambda(TreeType, Calls<'a>),
    Decorator(TreeType, Arguments<'a>, Box<Call<'a>>),
}

impl<'a> Call<'a> {
    pub fn invocation(id: &'a str, args: Arguments<'a>) -> Self {
        Call::Invocation(Id(id), args)
    }
    pub fn lambda(tpe: TreeType, calls: Calls<'a>) -> Self {
        Call::Lambda(tpe, calls)
    }
    pub fn decorator(tpe: TreeType, args: Arguments<'a>, call: Call<'a>) -> Self {
        Call::Decorator(tpe, args, Box::new(call))
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Calls<'a> {
    pub elems: Vec<Call<'a>>,
}

impl<'a> Calls<'a> {
    pub fn new(elems: Vec<Call<'a>>) -> Self {
        Calls { elems }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Param<'a> {
    pub name: Id<'a>,
    pub tpe: MesType,
}

impl<'a> Param<'a> {
    fn new(id: &'a str, tpe: MesType) -> Self {
        Param { name: Id(id), tpe }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Params<'a> {
    pub params: Vec<Param<'a>>,
}

impl<'a> Params<'a> {
    pub fn new(params: Vec<Param<'a>>) -> Self {
        Params { params }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Argument<'a> {
    Id(Id<'a>),
    Mes(Message<'a>),
    AssignedId(Id<'a>, Id<'a>),
    AssignedMes(Id<'a>, Message<'a>),
}

impl<'a> Argument<'a> {
    pub fn id(v: &'a str) -> Self {
        Argument::Id(Id(v))
    }
    pub fn mes(v: Message<'a>) -> Self {
        Argument::Mes(v)
    }
    pub fn id_id(lhs: &'a str, rhs: &'a str) -> Self {
        Argument::AssignedId(Id(lhs), Id(rhs))
    }
    pub fn id_mes(lhs: &'a str, rhs: Message<'a>) -> Self {
        Argument::AssignedMes(Id(lhs), rhs)
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Arguments<'a> {
    pub args: Vec<Argument<'a>>,
}

impl<'a> Arguments<'a> {
    pub fn new(args: Vec<Argument<'a>>) -> Self {
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
pub fn validate_lambda<'a,'b>(tpe: &'a TreeType, args: &'a Arguments<'a>, calls: &'a Calls<'a>) -> Result<(), &'b str> {
    match tpe{
        TreeType::Impl | TreeType::Cond => Err("the types impl or cond should have declaration and get called by name"),

        _ if tpe.is_decorator() => {
            if calls.elems.len() != 1 {
                Err("any decorator should have only one child")
            } else {
                Ok(())
            }
        }

        _ => {
            if args.args.is_empty(){
                Ok(())
            }else{
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