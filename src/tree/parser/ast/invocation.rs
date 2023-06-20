use crate::tree::parser::ast::arg::Arguments;
use crate::tree::parser::ast::Tree;
use crate::tree::project::AliasName;

/// Just a gathering structure of definition arguments and alias, representing a call
#[derive(Debug, Clone)]
pub struct Invocation<'a> {
    pub tree: &'a Tree,
    pub alias: Option<AliasName>,
    pub arguments: Arguments,
}

impl<'a> Invocation<'a> {
    pub fn new(tree: &'a Tree, arguments: Arguments) -> Self {
        Self {
            tree,
            alias: None,
            arguments,
        }
    }

    pub fn new_with_alias(tree: &'a Tree, alias: AliasName, arguments: Arguments) -> Self {
        Self {
            tree,
            alias: Some(alias),
            arguments,
        }
    }
}

impl<'a> From<&'a Tree> for Invocation<'a> {
    fn from(value: &'a Tree) -> Self {
        Invocation::new(value, Arguments::default())
    }
}
