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


impl<'a> From<&'a Tree> for Invocation<'a>{
    fn from(value: &'a Tree) -> Self {
        Invocation::new(value,Arguments::default())
    }
}