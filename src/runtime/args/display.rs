use crate::runtime::args::{RtArgs, RtArgument, RtValue, RtValueNumber};
use crate::tree::parser::ast::call::Call;
use itertools::Itertools;
use std::fmt::{Display, Formatter};

/// The short version of arguments to display,
/// It shows [..] and {..} when the list is too long
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
