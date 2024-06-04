use crate::runtime::args::{RtArgs, RtArgument, RtValue};
use crate::runtime::rtree::rnode::DecoratorType;
use crate::tree::parser::ast::arg::{
    Argument, ArgumentRhs, Arguments, ArgumentsType, Param, Params,
};
use crate::tree::parser::ast::message::Message;
use crate::tree::parser::ast::Key;
use crate::tree::{cerr, TreeError};
use std::collections::HashMap;

/// It converts the argument to the runtime argument
/// The parent attributes  are used to find the arguments
/// that comes from parents as pointer the from `parent(x:num) retry(x) action()`
// TODO the refactor the parent_args and parent_params to be a single struct (i am pretty sure i do the same thing in the other places)
fn dec_rt_arg(
    a: &ArgumentRhs,
    parent_args: Arguments,
    parent_params: Params,
) -> Result<RtValue, TreeError> {
    match a {
        ArgumentRhs::Id(p) => {
            // check of the parent has the argument with the given key
            // and if so then take it from there otherwise it is a pointer to bb
            match find_arg_value(p, &parent_params, &parent_args).ok() {
                None => Ok(RtValue::Pointer(p.to_string())),
                Some(v) => dec_rt_arg(&v, Arguments::default(), Params::default()),
            }
        }
        ArgumentRhs::Mes(Message::Num(n)) => Ok(RtValue::Number((*n).into())),
        e => Err(cerr(format!(
            "decorator has only one argument and it is either id or num but got {}",
            e
        ))),
    }
}

/// It extracts and validates the arguments for decorators since the contract is fixed.
/// The parent attributes  are used to find the arguments
/// that comes from parents as pointer the from `parent(x:num) retry(x) action()`
pub fn to_dec_rt_args(
    tpe: &DecoratorType,
    args: Arguments,
    parent_args: Arguments,
    parent_params: Params,
) -> Result<RtArgs, TreeError> {
    let empty = |args: &Arguments| {
        if args.args.is_empty() {
            Ok(RtArgs::default())
        } else {
            Err(cerr("decorator does not have arguments".to_string()))
        }
    };
    let one_num = |args: &Arguments| match args.args.as_slice() {
        [a] => {
            let v = dec_rt_arg(a.value(), parent_args, parent_params)?;
            Ok(RtArgs(vec![RtArgument::new_noname(v)]))
        }
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

/// It extracts and validates the arguments for decorators since the contract is fixed.
/// The parent attributes  are used to find the arguments
/// that comes from parents as pointer the from `parent(x:num) retry(x) action()`
/// returns the runtime arguments and the arguments that get updated (unfolded pointers)
pub fn to_rt_args(
    name: &str,
    args: Arguments,
    params: Params,
    p_args: Arguments,
    p_params: Params,
) -> Result<(RtArgs, Arguments), TreeError> {
    let mut rt_args: Vec<RtArgument> = vec![];
    match args.get_type()? {
        // we can't traverse the parameters if some of them are skipped
        ArgumentsType::Unnamed if args.args.len() != params.params.len() => {
            Err(cerr(format!(
                "the call {} doesn't have the same number of arguments and parameters",
                name
            )))
        }
        // find by the index according to the parameters
        ArgumentsType::Unnamed => {
            let mut upd_args = vec![];
            for (a, p) in args.args.into_iter().zip(params.params) {
                // if the that is a pointer we need to check parent also.
                let rhs = a.value().clone();
                let (rt_arg, upd_rhs) = RtArgument::try_from(rhs, p, p_args.clone(), p_params.clone())
                    .map_err(|r| r.modify(|s| format!("tree: {}, {}", name, s)))?;
                rt_args.push(rt_arg);
                upd_args.push(Argument::Unassigned(upd_rhs));
            }
            Ok((RtArgs(rt_args), Arguments::new(upd_args)))
        }
        // find by the name according to the parameters
        ArgumentsType::Named => {
            let mut upd_args = vec![];
            let param_map: HashMap<String, Param> =
                HashMap::from_iter(params.params.into_iter().map(|p| (p.name.clone(), p)));

            for a in args.args {
                let p = a.name().and_then(|n| param_map.get(n)).ok_or(cerr(format!(
                    "the argument {a} does not correspond to the definition"
                )))?;
                // if the that is a pointer we need to check parent also.
                let key = a.name().unwrap().to_string();
                let rhs = a.value().clone();
                let (rt_arg, upd_rhs) =
                    RtArgument::try_from(rhs, p.clone(), p_args.clone(), p_params.clone())
                        .map_err(|r| r.modify(|s| format!("tree: {}, {}", name, s)))?;
                rt_args.push(rt_arg);
                upd_args.push(Argument::Assigned(key, upd_rhs));
            }
            Ok((RtArgs(rt_args), Arguments::new(upd_args)))
        }
        ArgumentsType::Empty => Ok((RtArgs(rt_args), args)),
    }
}

/// find the argument value by the key
/// if the arguments are unnamed then the key is the index
/// if the arguments are named then the key is the name
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
