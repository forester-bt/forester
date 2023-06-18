use crate::runtime::RuntimeErrorCause;
use crate::tree::parser::ast::{Argument, ArgumentRhs, Arguments, ArgumentsType, Key, Params};
use crate::tree::{cerr, TreeError};

pub fn find_rhs_arg(
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
            .ok_or(cerr(format!("the argument can not be found {}", key))),
        ArgumentsType::Named => args
            .args
            .iter()
            .find(|a| a.has_name(key))
            .map(Argument::value)
            .ok_or(RuntimeErrorCause::arg(format!("no argument with the name {key}")).into())
            .cloned(),
        ArgumentsType::Empty => {
            Err(RuntimeErrorCause::arg("the arguments are empty".to_string()).into())
        }
    }
}
