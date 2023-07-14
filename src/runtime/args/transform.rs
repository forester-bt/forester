use crate::runtime::args::{RtArgs, RtArgument, RtValue, RtValueNumber};
use crate::runtime::rtree::rnode::DecoratorType;
use crate::tree::parser::ast::arg::{
    Argument, ArgumentRhs, Arguments, ArgumentsType, Param, Params,
};
use crate::tree::parser::ast::message::Message;
use crate::tree::parser::ast::Key;
use crate::tree::{cerr, TreeError};
use std::collections::HashMap;

/// It extracts and validates the arguments for decorators since the contract is fixed.
pub fn to_dec_rt_args(tpe: &DecoratorType, args: Arguments) -> Result<RtArgs, TreeError> {
    let empty = |args: &Arguments| {
        if args.args.is_empty() {
            Ok(RtArgs::default())
        } else {
            Err(cerr("decorator does not have arguments".to_string()))
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

            e => Err(cerr(format!(
                "decorator has only one argument and it is either id or num but got {}",
                e
            ))),
        },
        _ => Err(cerr("decorator has only one argument".to_string())),
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

pub fn to_rt_args(name: &str, args: Arguments, params: Params) -> Result<RtArgs, TreeError> {
    if args.args.len() != params.params.len() {
        Err(cerr(format!(
            "the call {} doesn't have the same number of arguments and parameters",
            name
        )))
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
                    let p = a.name().and_then(|n| param_map.get(n)).ok_or(cerr(format!(
                        "the argument {a} does not correspond to the definition"
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

pub fn find_arg_value(
    key: &Key,
    params: &Params,
    args: &Arguments,
) -> Result<ArgumentRhs, TreeError> {
    match &args.get_type()? {
        ArgumentsType::Unnamed => params
            .params
            .iter()
            .position(|p| p.name.as_str() == key)
            .and_then(|idx| args.args.get(idx))
            .map(Argument::value)
            .map(Clone::clone)
            .ok_or(cerr(format!("the argument can not be found {key}"))),
        ArgumentsType::Named => args
            .args
            .iter()
            .find(|a| a.has_name(key))
            .map(Argument::value)
            .ok_or(cerr(format!("no argument with the name {key}")))
            .cloned(),
        ArgumentsType::Empty => Err(cerr("the arguments are empty".to_string())),
    }
}
