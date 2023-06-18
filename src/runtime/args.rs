use crate::runtime::blackboard::BBKey;
use crate::runtime::rnode::DecoratorType;
use crate::runtime::RuntimeErrorCause;
use crate::tree::parser::ast::{
    validate_type, Argument, ArgumentRhs, Arguments, ArgumentsType, Call, MesType, Message, Number,
    Param, Params,
};
use itertools::Itertools;
use std::collections::HashMap;
use std::fmt::{Display, Formatter};
pub type RtAKey = String;
#[derive(Debug)]
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
#[derive(Debug)]
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

impl Display for RtValue {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            RtValue::Number(v) => match v {
                RtValueNumber::Int(v) => write!(f, "{}", v),
                RtValueNumber::Float(v) => write!(f, "{}", v),
                RtValueNumber::Hex(v) => write!(f, "{}", v),
                RtValueNumber::Binary(v) => write!(f, "{}", v),
            },
            RtValue::String(v) => write!(f, "{}", v),
            RtValue::Bool(b) => write!(f, "{b}"),
            RtValue::Array(array) => {
                let mut list = f.debug_list();
                let strings: Vec<_> = array.iter().map(|v| format!("{}", v)).collect();
                list.entries(strings);
                list.finish()
            }
            RtValue::Object(obj) => {
                let mut map = f.debug_map();
                let entries: Vec<_> = obj.iter().map(|(k, v)| (k, format!("{}", v))).collect();
                map.entries(entries);
                map.finish()
            }
            RtValue::Pointer(p) => write!(f, "&{p}"),
            RtValue::Call(c) => match c {
                Call::Invocation(t, _) => write!(f, "{}(..)", t),
                Call::InvocationCapturedArgs(t) => write!(f, "{}(..)", t),
                Call::Lambda(t, _) => write!(f, "{}..", t),
                Call::Decorator(t, _, _) => write!(f, "{}(..)", t),
            },
        }
    }
}

#[derive(Default, Debug)]
pub struct RtArgs(Vec<RtArgument>);
#[derive(Debug)]
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

    pub fn try_from(a: ArgumentRhs, p: Param) -> Result<Option<RtArgument>, RuntimeErrorCause> {
        let _ = validate_type(a.clone(), p.tpe)?;
        match &a {
            ArgumentRhs::Id(id) => Ok(Some(RtArgument::new(p.name, RtValue::Pointer(id.clone())))),
            ArgumentRhs::Mes(m) => Ok(Some(RtArgument::new(p.name, m.clone().into()))),
            ArgumentRhs::Call(c) => Ok(Some(RtArgument::new(p.name, RtValue::Call(c.clone())))),
        }
    }
}

#[derive(Debug)]
pub struct ShortDisplayedRtArguments<'a>(pub &'a RtArgs);

#[derive(Debug)]
pub struct ShortDisplayedRtArgument<'a>(pub &'a RtArgument);

impl<'a> Display for ShortDisplayedRtArgument<'a> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let short_mes = |m: &RtValue| match m {
            RtValue::Array(_) => "[..]".to_string(),
            RtValue::Object(_) => "{..}".to_string(),
            m => format!("{}", m),
        };

        let RtArgument { name, value } = &self.0;
        write!(f, "{}={}", name, short_mes(value))
    }
}

impl<'a> Display for ShortDisplayedRtArguments<'a> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let str = &self
            .0
             .0
            .iter()
            .map(|a| ShortDisplayedRtArgument(a))
            .map(|a| format!("{}", a))
            .join(",");
        if str.is_empty() {
            write!(f, "")
        } else {
            write!(f, "({})", str)
        }
    }
}

pub fn decorator_args(tpe: &DecoratorType, args: Arguments) -> Result<RtArgs, RuntimeErrorCause> {
    let empty = |args: &Arguments| {
        if args.args.is_empty() {
            Ok(RtArgs::default())
        } else {
            Err(RuntimeErrorCause::arg(
                "decorator does not have arguments".to_string(),
            ))
        }
    };
    let one_num = |args: &Arguments| match args.args.as_slice() {
        [a] => match a.value() {
            ArgumentRhs::Id(id) => Ok(RtArgs(vec![RtArgument::new_noname(RtValue::Pointer(
                id.to_string(),
            ))])),
            ArgumentRhs::Mes(Message::Num(n)) => {
                let r_num: RtValueNumber = (*n).into();
                Ok(RtArgs(vec![RtArgument::new_noname(RtValue::Number(r_num))]))
            }

            e => Err(RuntimeErrorCause::arg(format!(
                "decorator has only one argument and it is either id or num but got {}",
                e
            ))),
        },
        _ => Err(RuntimeErrorCause::arg(
            "decorator has only one argument".to_string(),
        )),
    };

    match tpe {
        DecoratorType::Inverter => empty(&args),
        DecoratorType::ForceSuccess => empty(&args),
        DecoratorType::ForceFail => empty(&args),
        DecoratorType::Repeat => one_num(&args),
        DecoratorType::Retry => one_num(&args),
        DecoratorType::Timeout => one_num(&args),
        DecoratorType::Delay => one_num(&args),
    }
}

pub fn invocation_args(args: Arguments, params: Params) -> Result<RtArgs, RuntimeErrorCause> {
    if args.args.len() != params.params.len() {
        Err(RuntimeErrorCause::arg(
            "don't match the contract".to_string(),
        ))
    } else {
        let mut rt_args: Vec<RtArgument> = vec![];
        match args.get_type()? {
            ArgumentsType::Unnamed => {
                for (a, p) in args.args.into_iter().zip(params.params) {
                    if let Some(rt_a) = RtArgument::try_from(a.value().clone(), p)? {
                        rt_args.push(rt_a);
                    }
                }
                Ok(RtArgs(rt_args))
            }
            ArgumentsType::Named => {
                let param_map: HashMap<String, Param> =
                    HashMap::from_iter(params.params.into_iter().map(|p| (p.name.clone(), p)));

                for a in args.args {
                    let p =
                        a.name()
                            .and_then(|n| param_map.get(n))
                            .ok_or(RuntimeErrorCause::arg(format!(
                                "the argument {} does not correspond to the definition",
                                a
                            )))?;
                    if let Some(rt_a) = RtArgument::try_from(a.value().clone(), p.clone())? {
                        rt_args.push(rt_a);
                    }
                }
                Ok(RtArgs(rt_args))
            }
            ArgumentsType::Empty => Ok(RtArgs(rt_args)),
        }
    }
}
