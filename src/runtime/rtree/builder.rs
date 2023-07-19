use crate::runtime::args::transform::find_arg_value;
use crate::runtime::RuntimeError;
use crate::tree::parser::ast::arg::{Arguments, Params};
use crate::tree::parser::ast::call::{Call, Calls};
use crate::tree::parser::ast::Key;
use crate::tree::{cerr, TreeError};
use std::collections::{HashMap, VecDeque};

/// The temp struct helps to transform ast to runtime tree
/// Basically, it needs to keep 2 things
///  - the bfs iter order stack  
///  - the stack trace of the calls
///
/// # Note
/// The internal structure
#[derive(Default)]
pub struct InternalBuilder {
    gen: usize,
    stack: VecDeque<StackItem>,
    chain_map: HashMap<usize, ChainItem>,
}
// to help in traversing the call stack
pub struct StackItem {
    pub id: usize,
    pub call: Call,
    pub parent_id: usize,
    pub file_name: String,
}

/// represents a parent of the call in the stack trace
pub enum ChainItem {
    Tree(usize, Arguments, Params),
    Lambda(usize),
    Root,
}

impl ChainItem {
    pub fn get_tree(&self) -> (usize, Arguments, Params) {
        match self {
            ChainItem::Tree(id, a, p) => (*id, a.clone(), p.clone()),
            _ => (0, Arguments::default(), Params::default()),
        }
    }
    pub fn is_lambda(&self) -> bool {
        match self {
            ChainItem::Lambda(_) => true,
            ChainItem::Tree(_, _, _) => false,
            ChainItem::Root => false,
        }
    }
    pub fn parent(&self) -> usize {
        match self {
            ChainItem::Tree(p, _, _) => *p,
            ChainItem::Lambda(p) => *p,
            ChainItem::Root => 0,
        }
    }
}

impl InternalBuilder {
    pub fn next(&mut self) -> usize {
        self.gen += 1;
        self.gen
    }

    pub fn get_chain(&self, id: &usize) -> Result<&ChainItem, TreeError> {
        if *id == 0 {
            Ok(&ChainItem::Root)
        } else {
            self.chain_map
                .get(id)
                .ok_or(cerr(format!("the item {id} falls out from the chain")))
        }
    }

    /// The method finds the next higher order of the call.
    /// ```norun
    ///  sequence tree_def(op:tree) another(n = op(..))
    ///  sequence another(n:tree) n(..)  
    /// ```
    /// To process `n(..)` in `another` we have to climb up after  `n=op(..)` until the last ho call.
    pub fn find_ho_call(&self, parent_id: &usize, key: &Key) -> Result<Call, TreeError> {
        let (mut grand_parent, mut parent_args, mut parent_params) =
            self.get_chain_skip_lambda(&parent_id)?.get_tree();

        let mut call = find_arg_value(&key, &parent_params, &parent_args)?.get_call();

        while let Some(key) = call.clone().and_then(|c| c.get_ho_invocation()) {
            (grand_parent, parent_args, parent_params) =
                self.get_chain_skip_lambda(&grand_parent)?.get_tree();
            call = find_arg_value(&key, &parent_params, &parent_args)?.get_call();
        }
        call.ok_or(cerr(format!("the argument {key} should be a tree")))
    }

    ///goes up on the stacktrace skipping lambda
    pub fn get_chain_skip_lambda(&self, id: &usize) -> Result<&ChainItem, TreeError> {
        let mut current = self.get_chain(id)?;

        while current.is_lambda() {
            current = self.get_chain(&current.parent())?;
        }

        Ok(current)
    }

    pub fn add_chain(&mut self, id: usize, parent: usize, args: Arguments, params: Params) {
        self.chain_map
            .insert(id, ChainItem::Tree(parent, args, params));
    }
    pub fn add_chain_root(&mut self, id: usize) {
        self.chain_map.insert(id, ChainItem::Root);
    }
    pub fn add_chain_lambda(&mut self, id: usize, parent: usize) {
        self.chain_map.insert(id, ChainItem::Lambda(parent));
    }

    pub fn push(&mut self, call: Call, parent_id: usize, file_name: String) -> usize {
        let id = self.next();
        self.stack.push_back(StackItem {
            id,
            call,
            parent_id,
            file_name,
        });
        id
    }
    pub fn push_vec(&mut self, calls: Calls, parent_id: usize, file_name: String) -> Vec<usize> {
        let mut children = vec![];
        for call in calls.elems {
            children.push(self.push(call, parent_id, file_name.clone()));
        }

        children
    }
    pub fn push_front(
        &mut self,
        id: usize,
        call: Call,
        parent_id: usize,
        file_name: String,
    ) -> usize {
        self.stack.push_front(StackItem {
            id,
            call,
            parent_id,
            file_name,
        });
        id
    }
    pub fn pop(&mut self) -> Option<StackItem> {
        self.stack.pop_front()
    }
}
