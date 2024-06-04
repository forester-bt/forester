use crate::tree::parser::ast::arg::Arguments;
use crate::tree::parser::ast::{Key, TreeType};
use serde::{Deserialize, Serialize};
use std::fmt::{Debug, Formatter};

/// A call to a tree
#[derive(Clone, PartialEq, Deserialize, Serialize)]
pub enum Call {
    /// An invocation of a tree like 'root main { invocation()}'
    Invocation(Key, Arguments),
    /// An Higher order invocation of a tree like 'root main { ho-invocation(..)}'
    HoInvocation(Key),
    /// A lambda call like 'root main { sequence {...} }'
    Lambda(TreeType, Calls),
    /// A decorator call like 'root main { decorator(..) child() }'
    Decorator(TreeType, Arguments, Box<Call>),
}

impl Debug for Call {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Call::Invocation(id, args) => write!(f, "{}({})", id, args),
            Call::HoInvocation(id) => write!(f, "{}(..)", id),
            Call::Lambda(tpe, calls) => {
                let _ = write!(f, "{} :", tpe);
                let mut elems = f.debug_list();
                for call in calls.elems.iter() {
                    elems.entry(call);
                }
                let _ = elems.finish();
                Ok(())
            }
            Call::Decorator(tpe, args, call) => {
                let _ = write!(f, "{}({}) :", tpe, args);
                let mut elems = f.debug_list();
                elems.entry(call);
                elems.finish();
                Ok(())
            }
        }
    }
}

impl Call {
    pub fn is_lambda(&self) -> bool {
        matches!(self, Call::Lambda(_, _))
    }
    pub fn is_decorator(&self) -> bool {
        matches!(self, Call::Decorator(_,_, _))
    }

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
