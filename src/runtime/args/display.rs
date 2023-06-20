use crate::runtime::args::{RtArgs, RtArgument, RtValue, RtValueNumber};
use crate::tree::parser::ast::call::Call;
use itertools::Itertools;
use std::fmt::{Display, Formatter};

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
                Call::HoInvocation(t) => write!(f, "{}(..)", t),
                Call::Lambda(t, _) => write!(f, "{}..", t),
                Call::Decorator(t, _, _) => write!(f, "{}(..)", t),
            },
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
            RtValue::Call(Call::Lambda(tpe, _)) => format!("{tpe}.."),
            RtValue::Call(Call::Decorator(tpe, args, _)) => format!("{tpe}({args})"),
            RtValue::Call(Call::Invocation(key, _)) => format!("{key}(<>)"),
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
