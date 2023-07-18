use crate::runtime::action::builtin::data::{CheckEq, LockUnlockBBKey, StoreData, StoreTick};
use crate::runtime::action::builtin::http::HttpGet;
use crate::runtime::action::builtin::ReturnResult;
use crate::runtime::action::keeper::ActionKeeper;
use crate::runtime::action::{Action, ActionName};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::env::RtEnv;
use crate::runtime::forester::Forester;
use crate::runtime::rtree::{RuntimeTree, RuntimeTreeStarter};
use crate::runtime::{RtResult, RuntimeError};
use crate::tracer::Tracer;
use crate::tree::project::{FileName, Project, TreeName};
use std::collections::HashMap;
use std::fmt::format;
use std::path::{Path, PathBuf};

/// The builder to create a Forester instance
///
///# Example
///```
/// use std::path::PathBuf;
/// use forester_rs::tracer::Tracer;
/// use forester_rs::runtime::builder::ForesterBuilder;
/// use forester_rs::runtime::action::Action;
/// use forester_rs::runtime::action::builtin::data::StoreData;
/// fn test(root:PathBuf){
///     let mut root = PathBuf::new();
///
///     let mut fb = ForesterBuilder::new();
///     fb.main_file("main.tree".to_string());
///     fb.root(root);
///     fb.register_action("store", Action::sync(StoreData));
///     
///     fb.tracer(Tracer::default());
///     fb.bb_load("db/db.json".to_string());
///     let forester = fb.build().unwrap();
/// }
/// fn test_from_text(){
///
///     let mut fb = ForesterBuilder::new();
///            
///     fb.text(r#"root main store("hello", "world") "#.to_string());    
///
///     fb.register_action("store", Action::sync(StoreData));
///     
///     let forester = fb.build().unwrap();
/// }
///
/// ```
pub struct ForesterBuilder {
    actions: HashMap<ActionName, Action>,
    main_file: Option<FileName>,
    main: Option<TreeName>,
    root: Option<PathBuf>,
    bb: BlackBoard,
    tracer: Tracer,
    env: Option<RtEnv>,
    bb_load: Option<String>,
    text: Option<String>,
}

impl ForesterBuilder {
    pub fn new() -> Self {
        Self {
            actions: HashMap::default(),
            main_file: None,
            main: None,
            root: None,
            bb: BlackBoard::default(),
            tracer: Tracer::noop(),
            bb_load: None,
            env: None,
            text: None,
        }
    }
    /// Add an action accroding to the name.
    pub fn register_action(&mut self, name: &str, action: Action) {
        self.actions.insert(name.to_string(), action);
    }

    /// add script on the fly.
    /// In that scenario, there is no need in the other attributes like files or root.
    ///
    /// Precautions:
    /// Imports and other files still work only with absolute paths.
    pub fn text(&mut self, txt: String) {
        self.text = Some(txt);
    }

    /// Root folder.
    pub fn root(&mut self, root: PathBuf) {
        self.root = Some(root);
    }
    /// A file that has a main root definition
    pub fn main_file(&mut self, main_file: FileName) {
        self.main_file = Some(main_file);
    }
    /// A name of the main root definition
    pub fn main_tree(&mut self, main_tree: TreeName) {
        self.main = Some(main_tree);
    }
    /// Tracer that will be used to save the tracing information.
    pub fn tracer(&mut self, tr: Tracer) {
        self.tracer = tr;
    }
    /// A file that has a snapshot of the bb in json format.
    pub fn bb_load(&mut self, bb: String) {
        self.bb_load = Some(bb);
    }

    /// Mix the runtime async env(tokio env)
    /// By default, creates the default tokio Runtime multi thread
    pub fn rt_env(&mut self, env: RtEnv) {
        self.env = Some(env);
    }

    /// The method to build forester
    pub fn build(self) -> RtResult<Forester> {
        let project = if let Some(t) = self.text.clone() {
            Project::build_from_text(t)?
        } else {
            match (self.main, self.root.clone(), self.main_file) {
                (None, Some(root), Some(mf)) => Project::build(mf, root)?,
                (Some(mt), Some(root), Some(mf)) => Project::build_with_root(mf, mt, root)?,
                _ => {
                    return Err(RuntimeError::UnImplementedAction(format!(
                        "not enough arguments to initialize the project"
                    )))
                }
            }
        };
        let RuntimeTreeStarter { tree, std_actions } = RuntimeTree::build(project)?;
        let mut actions = self.actions;

        for action_name in std_actions.iter() {
            let action = BuilderBuiltInActions::action_impl(action_name)?;
            actions.insert(action_name.clone(), action);
        }

        let mut bb = BlackBoard::default();
        if let Some(bb_load_dump) = self.bb_load {
            let file = PathBuf::from(bb_load_dump);
            let file = if file.is_relative() {
                if self.text.is_some() && self.root.is_none() {
                    return Err(RuntimeError::Unexpected(
                        format!("There is impossible to have a relative path for bb_load since the root is absent and the option from text is setup.")
                    ));
                }
                let mut r = self.root.unwrap().clone();
                r.push(file);
                r
            } else {
                file
            };
            bb.load(&file)?;
        };

        let env = if let Some(e) = self.env {
            e
        } else {
            RtEnv::try_new()?
        };

        Forester::new(
            tree,
            BlackBoard::default(),
            ActionKeeper::new(actions)?,
            self.tracer,
            env,
        )
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
            "http_get" => Ok(Action::sync(HttpGet)),
            "http_get_async" => Ok(Action::a_sync(HttpGet)),
            "lock" => Ok(Action::sync(LockUnlockBBKey::Lock)),
            "unlock" => Ok(Action::sync(LockUnlockBBKey::Unlock)),

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

/// Performs http get request
impl http_get(url:string, bb_key:string);

/// Performs http get request
impl http_get_async(url:string, bb_key:string);

// Lock key in bb
impl lock(key:string);

// Unlock key in bb
impl unlock(key:string);

"#
        .to_string()
    }
}
