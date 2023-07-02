use crate::runtime::action::builtin::data::{CheckEq, StoreData, StoreTick};
use crate::runtime::action::builtin::ReturnResult;
use crate::runtime::action::keeper::ActionKeeper;
use crate::runtime::action::{Action, ActionName};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::forester::Forester;
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::{RtResult, RuntimeError};
use crate::tree::project::{FileName, Project, TreeName};
use std::collections::HashMap;
use std::fmt::format;
use std::path::{Path, PathBuf};

pub struct ForesterBuilder {
    actions: HashMap<ActionName, Action>,
    main_file: Option<FileName>,
    main: Option<TreeName>,
    root: Option<PathBuf>,
    bb: BlackBoard,
}

impl ForesterBuilder {
    pub fn new() -> Self {
        Self {
            actions: HashMap::default(),
            main_file: None,
            main: None,
            root: None,
            bb: BlackBoard::default(),
        }
    }

    pub fn register_action(&mut self, name: &str, action: Action) {
        self.actions.insert(name.to_string(), action);
    }

    pub fn root(&mut self, root: PathBuf) {
        self.root = Some(root);
    }
    pub fn main_file(&mut self, main_file: FileName) {
        self.main_file = Some(main_file);
    }
    pub fn main_tree(&mut self, main_tree: TreeName) {
        self.main = Some(main_tree);
    }

    pub fn build(self) -> RtResult<Forester> {
        let project = match (self.main, self.root, self.main_file) {
            (None, Some(root), Some(mf)) => Project::build(mf, root)?,
            (Some(mt), Some(root), Some(mf)) => Project::build_with_root(mf, mt, root)?,
            _ => {
                return Err(RuntimeError::UnImplementedAction(format!(
                    "not enough arguments to initialize the project"
                )))
            }
        };
        let tree = RuntimeTree::build(project)?;
        let mut actions = self.actions;

        for action_name in tree.std_nodes.iter() {
            let action = BuilderBuiltInActions::action_impl(action_name)?;
            actions.insert(action_name.clone(), action);
        }

        Forester::new(tree, BlackBoard::default(), ActionKeeper::new(actions))
    }
}

pub struct BuilderBuiltInActions;

impl BuilderBuiltInActions {
    fn action_impl(action: &ActionName) -> RtResult<Action> {
        match action.as_str() {
            "fail_empty" => Ok(Action::sync(ReturnResult::fail_empty())),
            "fail" => Ok(Action::sync(ReturnResult::fail_empty())),
            "success" => Ok(Action::sync(ReturnResult::success())),
            "running" => Ok(Action::sync(ReturnResult::running())),
            "store_str" => Ok(Action::sync(StoreData)),
            "eq_str" => Ok(Action::sync(CheckEq)),
            "eq_num" => Ok(Action::sync(CheckEq)),
            "store_tick" => Ok(Action::sync(StoreTick)),

            _ => Err(RuntimeError::UnImplementedAction(format!(
                "action {action} is absent in the library"
            ))),
        }
    }

    pub fn builtin_actions_file() -> String {
        r#"
//
// Built-in actions. 
// The actions are accessible using the import 'import "std::actions"' 
// Better off, the file be avoided modifying
//

// Fails execution, returning Result::Failure        
impl fail(reason:string);
impl fail_empty();

// Success execution, returning Result::Success  
impl success();

// Running execution, returning Result::Running  
impl running();

// Sleeps on duration(milliseconds) then returns Result::Success
// impl sleep(duration:num);

// Stores the string value in the given key. Returns Result::Success. 
// If the cell is locked, returns Result::Failure   
impl store_str(key:string, value:string);

// Compares given string value with what is in the cell:
// - Returns Result::Success if they are equal
// - Returns Fail(reason)if they are not equal
// - Returns Fail(reason) if there is no cell in bbe with the given key.
impl eq_str(key:string, expected:string);
impl eq_num(key:string, expected:num);

/// Store the current tick
impl store_tick(name:string);

"#
        .to_string()
    }
}
