use std::fmt::format;
use itertools::Itertools;
use crate::tree::parser::ast::{Argument, Arguments, Call, MesType, Params, Tree};
use crate::tree::project::AliasName;
use crate::tree::{cerr, TreeError};

#[derive(Debug, Clone)]
pub struct Invocation<'a> {
    pub tree: &'a Tree,
    pub alias: Option<AliasName>,
    pub arguments: Arguments,
}

impl<'a> Invocation<'a> {
    pub fn new(tree: &'a Tree, arguments: Arguments) -> Self {
        Self { tree, alias: None, arguments }
    }

    pub fn new_with_alias(tree: &'a Tree, alias: AliasName, arguments: Arguments) -> Self {
        Self { tree, alias: Some(alias), arguments }
    }
}

fn validate_arguments(params: &Params, arguments: &Arguments) -> Result<(), TreeError> {
    #[derive(PartialEq,Copy,Clone)]
    enum ArgType {
        Named,
        Unnamed,
    }

    let mut tpe: Option<ArgType> = None;

    for arg in arguments.args.iter() {
        match arg {
            Argument::Id(_) | Argument::Mes(_) => {
                if tpe.is_none() {
                    tpe = Some(ArgType::Unnamed);
                } else {
                    if tpe.unwrap() != ArgType::Unnamed {
                        return Err(cerr(format!("the arguments should be either unnamed or named")));
                    }
                }
            }
            Argument::AssignedId(_, _) | Argument::AssignedMes(_, _) => {
                if tpe.is_none() {
                    tpe = Some(ArgType::Named);
                } else {
                    if tpe.unwrap() != ArgType::Named {
                        return Err(cerr(format!("the arguments should be either unnamed or named")));
                    }
                }
            }
        }
    }

    if params.params.len() != arguments.args.len() {
        return Err(cerr(format!("the arguments correspond to the contract")));
    }

    for (i, p) in params.params.iter().enumerate() {
        match &tpe {
            None => {
                return Err(cerr(format!("unexpected error with params")));
            }
            Some(ArgType::Unnamed) => {
                let arg = arguments.args.get(i).unwrap();
                match arg {
                    Argument::Mes(m) if !m.same(&p.tpe) => {
                        return Err(cerr(format!("the argument {:?} has a wrong type.", arg)));
                    }
                    _ => {}
                }
            }
            Some(ArgType::Named) => {
                let arg = arguments.args.iter().find(|f| f.name().filter(|f| *f == &p.name).is_some());
                if let Some(arg)  = arg {
                    match arg {
                        Argument::AssignedMes(_,m) if !m.same(&p.tpe) => {
                            return Err(cerr(format!("the argument {:?} has a wrong type.", arg)));
                        }
                        _ => {}
                    }
                } else {
                    return Err(cerr(format!("the argument with the name {} is not found",p.name)))
                }
            }
        }
    }


    Ok(())
}