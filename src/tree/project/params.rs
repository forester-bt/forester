use crate::tree::parser::ast::{Argument, ArgumentRhs, Arguments, Key, Params};
use crate::tree::{cerr, TreeError};

pub fn find_arg(key: &Key, params: &Params, args: &Arguments) -> Result<ArgumentRhs, TreeError> {
    let named_arg =
        args
            .args
            .iter()
            .find(|a| a.has_name(key))
            .map(Argument::value);

    if named_arg.is_some() {
        Ok(named_arg.unwrap().clone())
    } else {
        params
            .params
            .iter()
            .position(|p| p.name.as_str() == key)
            .and_then(|idx| args.args.get(idx))
            .map(Argument::value)
            .map(Clone::clone)
            .ok_or(cerr(format!("the argument can not be found {}", key)))
    }
}