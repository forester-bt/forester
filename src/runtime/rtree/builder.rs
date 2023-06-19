use crate::tree::parser::ast::{Arguments, Call, Calls, Params};
use crate::tree::TreeError;
use std::collections::{HashMap, VecDeque};

#[derive(Default)]
pub struct Builder {
    gen: usize,
    stack: VecDeque<StackItem>,
    chain_map: HashMap<usize, ChainItem>,
}

pub struct StackItem {
    pub id: usize,
    pub call: Call,
    pub parent_id: usize,
    pub file_name: String,
}

pub enum ChainItem {
    Tree(usize, Arguments, Params),
    Lambda(usize),
    Root,
}

impl ChainItem {
    pub fn get_args(&self) -> (Arguments, Params) {
        match self {
            ChainItem::Tree(_, a, p) => (a.clone(), p.clone()),
            _ => (Arguments::default(), Params::default()),
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

impl Builder {
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
                .ok_or(TreeError::CompileError(format!(
                    "the item {} falls out from the chain",
                    id
                )))
        }
    }

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
