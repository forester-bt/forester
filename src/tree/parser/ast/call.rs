use crate::tree::parser::ast::arg::Arguments;
use crate::tree::parser::ast::{Key, TreeType};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
pub enum Call {
    Invocation(Key, Arguments),
    HoInvocation(Key),
    Lambda(TreeType, Calls),
    Decorator(TreeType, Arguments, Box<Call>),
}

impl Call {
    pub fn get_ho_invocation(&self) -> Option<Key> {
        match self {
            Call::HoInvocation(k) => Some(k.clone()),
            _ => None,
        }
    }

    pub fn key(&self) -> Option<Key> {
        match self {
            Call::Invocation(k, _) => Some(k.clone()),
            Call::HoInvocation(k) => Some(k.clone()),
            Call::Lambda(_, _) => None,
            Call::Decorator(_, _, _) => None,
        }
    }
    pub fn arguments(&self) -> Arguments {
        match self {
            Call::Invocation(_, args) => args.clone(),
            Call::HoInvocation(_) => Arguments::default(),
            Call::Lambda(_, _) => Arguments::default(),
            Call::Decorator(_, args, _) => args.clone(),
        }
    }

    pub fn invocation(id: &str, args: Arguments) -> Self {
        Call::Invocation(id.to_string(), args)
    }
    pub fn ho_invocation(id: &str) -> Self {
        Call::HoInvocation(id.to_string())
    }
    pub fn lambda(tpe: TreeType, calls: Calls) -> Self {
        Call::Lambda(tpe, calls)
    }
    pub fn decorator(tpe: TreeType, args: Arguments, call: Call) -> Self {
        Call::Decorator(tpe, args, Box::new(call))
    }
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
pub struct Calls {
    pub elems: Vec<Call>,
}

impl Calls {
    pub fn new(elems: Vec<Call>) -> Self {
        Calls { elems }
    }
}
