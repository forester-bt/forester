pub mod arg;
pub mod call;
pub mod invocation;
pub mod message;

use crate::runtime::rnode::Name;
use crate::runtime::RuntimeErrorCause;
use crate::tree::parser::ast::invocation::Invocation;
use crate::tree::project::{AliasName, TreeName};
use crate::tree::TreeError;
use arg::ArgumentsType::{Named, Unnamed};
use arg::{Arguments, Params};
use call::{Call, Calls};
use itertools::Itertools;
use message::Message;
use parsit::step::Step;
use std::collections::HashMap;
use std::fmt::{format, write, Display, Formatter, Write};
use std::str::FromStr;
use strum::ParseError;
use strum_macros::Display;
use strum_macros::EnumString;

pub type Key = String;

#[derive(Display, Debug, Clone, Copy, Eq, PartialEq, EnumString)]
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
    Delay,
    // actions
    Impl,
    Cond,
}

impl TreeType {
    pub fn is_decorator(&self) -> bool {
        match self {
            TreeType::Inverter
            | TreeType::ForceSuccess
            | TreeType::ForceFail
            | TreeType::Repeat
            | TreeType::Retry
            | TreeType::Timeout => true,
            _ => false,
        }
    }
    pub fn is_action(&self) -> bool {
        match self {
            TreeType::Impl | TreeType::Cond => true,
            _ => false,
        }
    }
}

pub fn validate_lambda<'a, 'b>(
    tpe: &'a TreeType,
    args: &'a Arguments,
    calls: &'a Calls,
) -> Result<(), &'b str> {
    match tpe {
        TreeType::Impl | TreeType::Cond => {
            Err("the types impl or cond should have declaration and get called by name")
        }

        _ if tpe.is_decorator() => {
            if calls.elems.len() != 1 {
                Err("any decorator should have only one child")
            } else {
                Ok(())
            }
        }

        _ => {
            if args.args.is_empty() {
                Ok(())
            } else {
                Err("any lambda invocation should not have arguments")
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Tree {
    pub tpe: TreeType,
    pub name: Key,
    pub params: Params,
    pub calls: Calls,
}

impl Tree {
    pub fn root(name: &str, calls: Vec<Call>) -> Self {
        Tree::new(
            TreeType::Root,
            name.to_string(),
            Params::default(),
            Calls::new(calls),
        )
    }
}

impl Tree {
    pub fn is_root(&self) -> bool {
        match self.tpe {
            TreeType::Root => true,
            _ => false,
        }
    }
    pub fn new(tpe: TreeType, name: Key, params: Params, calls: Calls) -> Self {
        Self {
            tpe,
            name,
            params,
            calls,
        }
    }
    pub fn to_inv(&self) -> Invocation {
        self.into()
    }
    pub fn to_inv_args(&self, args: Arguments) -> Invocation {
        Invocation::new(self, args)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Import(pub String, pub Vec<ImportName>);

#[derive(Clone, Debug, PartialEq, Hash, Eq)]
pub enum ImportName {
    Id(String),
    Alias(TreeName, AliasName),
    WholeFile,
}

impl ImportName {
    pub fn id(v: &str) -> Self {
        ImportName::Id(v.to_string())
    }
    pub fn alias(v: &str, alias: &str) -> Self {
        ImportName::Alias(v.to_string(), alias.to_string())
    }
}

impl Import {
    pub fn f_name(&self) -> &str {
        match self {
            Import(n, _) => n,
        }
    }
    pub fn file(f: &str) -> Self {
        Import(f.to_string(), vec![ImportName::WholeFile])
    }
    pub fn names(f: &str, names: Vec<&str>) -> Self {
        Import(
            f.to_string(),
            names.into_iter().map(|v| ImportName::id(v)).collect(),
        )
    }
    pub fn names_mixed(f: &str, names: Vec<ImportName>) -> Self {
        Import(f.to_string(), names)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum FileEntity {
    Tree(Tree),
    Import(Import),
}

#[derive(Clone, Debug, PartialEq)]
pub struct AstFile(pub Vec<FileEntity>);

impl<'a> AstFile {
    pub fn new(field0: Vec<FileEntity>) -> Self {
        Self(field0)
    }
}
