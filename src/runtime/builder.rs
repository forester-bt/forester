pub mod builtin;
pub mod custom_builder;
pub mod file_builder;
pub mod text_builder;
pub mod ros_nav;
pub mod ros_core;

use crate::get_pb;

use crate::runtime::action::keeper::{ActionImpl, ActionKeeper};
use crate::runtime::action::{Action, ActionName, Impl, ImplAsync, ImplRemote};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::builder::custom_builder::CustomForesterBuilder;
use crate::runtime::builder::file_builder::FileForesterBuilder;
use crate::runtime::builder::text_builder::TextForesterBuilder;
use crate::runtime::env::RtEnv;

use crate::runtime::forester::{serv, Forester};
use crate::runtime::rtree::builder::RtNodeBuilder;
use crate::runtime::rtree::rnode::RNodeId;
use crate::runtime::rtree::{RuntimeTree, RuntimeTreeStarter};
use crate::runtime::{RtOk, RtResult, RuntimeError};
use crate::tracer::Tracer;
use crate::tree::project::{FileName, TreeName};

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use crate::runtime::env::daemon::{DaemonName, Daemon};
use crate::runtime::env::daemon::context::DaemonContext;

/// The builder to create a Forester instance
///
///# Example
///```
/// use std::path::PathBuf;
/// use forester_rs::tracer::Tracer;
/// use forester_rs::runtime::builder::ForesterBuilder;
/// use forester_rs::runtime::action::Action;
/// use forester_rs::runtime::action::builtin::data::StoreData;
/// use forester_rs::runtime::rtree::builder::RtNodeBuilder;
/// use forester_rs::*;
/// use forester_rs::runtime::rtree::rnode::{DecoratorType, FlowType, RNodeName};
/// use forester_rs::runtime::args::RtArgs;
/// fn from_file(root:PathBuf){
///     let mut fb = ForesterBuilder::from_fs();
///     fb.main_file("main.tree".to_string());
///     fb.root(root);
///     fb.register_sync_action("store", StoreData);
///     
///     fb.tracer(Tracer::default());
///     fb.bb_load("db/db.json".to_string());
///     let forester = fb.build().unwrap();
/// }
/// fn from_text(){
///
///     let mut fb = ForesterBuilder::from_text();
///            
///     fb.text(r#"root main store("hello", "world") "#.to_string());    
///
///     fb.register_sync_action("store", StoreData);
///     
///     let forester = fb.build().unwrap();
/// }
/// fn from_code(){
///
///     let mut fb = ForesterBuilder::from_code();
///            
///     fb.add_rt_node(
///          flow!(fallback node_name!(), args!();
///                         action!(),
///                         action!(),
///                         action!(),
///                         action!()
///                     )
///      );
///
///     fb.register_sync_action("store", StoreData);
///     
///     let forester = fb.build().unwrap();
/// }
///
/// ```
pub enum ForesterBuilder {
    Files {
        cfb: CommonForesterBuilder,
        delegate: FileForesterBuilder,
        error: Option<String>,
    },
    Text {
        cfb: CommonForesterBuilder,
        delegate: TextForesterBuilder,
        error: Option<String>,
    },
    Code {
        cfb: CommonForesterBuilder,
        delegate: CustomForesterBuilder,
        error: Option<String>,
    },
}

impl ForesterBuilder {
    pub fn from_fs() -> ForesterBuilder {
        ForesterBuilder::Files {
            cfb: CommonForesterBuilder::new(),
            delegate: FileForesterBuilder::new(),
            error: None,
        }
    }
    pub fn from_text() -> ForesterBuilder {
        ForesterBuilder::Text {
            cfb: CommonForesterBuilder::new(),
            delegate: TextForesterBuilder::new(),
            error: None,
        }
    }
    pub fn from_code() -> ForesterBuilder {
        ForesterBuilder::Code {
            cfb: CommonForesterBuilder::new(),
            delegate: CustomForesterBuilder::new(),
            error: None,
        }
    }

    /// Root folder.
    pub fn root(&mut self, root: PathBuf) {
        match self {
            ForesterBuilder::Files { delegate, .. } => {
                delegate.root(root);
            }
            ForesterBuilder::Text { error, .. } | ForesterBuilder::Code { error, .. } => {
                let _ = error.insert("This type of builder does not accept root folder. Only `from_file_system` builder accept it.".to_string());
            }
        }
    }
    /// A file that has a main root definition
    pub fn main_file(&mut self, main_file: FileName) {
        match self {
            ForesterBuilder::Files { delegate, .. } => {
                delegate.main_file(main_file);
            }
            ForesterBuilder::Text { error, .. } | ForesterBuilder::Code { error, .. } => {
                let _ = error.insert("This type of builder does not accept main_file. Only `from_file_system` builder accept it.".to_string());
            }
        }
    }
    /// A name of the main root definition
    pub fn main_tree(&mut self, main_tree: TreeName) {
        match self {
            ForesterBuilder::Files { delegate, .. } => {
                delegate.main_tree(main_tree);
            }
            ForesterBuilder::Text { error, .. } | ForesterBuilder::Code { error, .. } => {
                let _ = error.insert("This type of builder does not accept main_tree. Only `from_file_system` builder accept it.".to_string());
            }
        }
    }

    /// add script on the fly.
    /// In that scenario, there is no need in the other attributes like files or root.
    ///
    /// Precautions:
    /// Imports and other files still work only with absolute paths.
    pub fn text(&mut self, txt: String) {
        match self {
            ForesterBuilder::Text { delegate, .. } => {
                delegate.text(txt);
            }
            ForesterBuilder::Files { error, .. } | ForesterBuilder::Code { error, .. } => {
                let _ = error.insert("This type of builder does not accept code as text. Only `from_text` builder accept it.".to_string());
            }
        }
    }
    /// add runtime node.
    pub fn add_rt_node(&mut self, node_b: RtNodeBuilder) -> RNodeId {
        match self {
            ForesterBuilder::Code { delegate, .. } => delegate.add_rt_node(node_b),
            ForesterBuilder::Files { error, .. } | ForesterBuilder::Text { error, .. } => {
                let _ = error.insert("This type of builder does not accept code as text. Only `from_text` builder accept it.".to_string());
                0
            }
        }
    }


    /// Add a daemon
    pub fn register_daemon(&mut self, daemon: Daemon) {
        self.cfb().register_daemon(daemon);
    }

    /// Add a daemon with a name
    /// The name is used to stop the daemon
    pub fn register_named_daemon(&mut self, name: DaemonName, daemon: Daemon) {
        self.cfb().register_named_daemon(name, daemon);
    }


    /// Add a sync action according to the name.
    pub fn register_sync_action<A>(&mut self, name: &str, action: A)
        where
            A: Impl + 'static,
    {
        self.cfb().register_sync_action(name, action);
    }
    /// Add an async action according to the name.
    pub fn register_async_action<A>(&mut self, name: &str, action: A)
        where
            A: ImplAsync + 'static,
    {
        self.cfb().register_async_action(name, action);
    }

    /// Add an action according to the name but with a promise the action remote.
    pub fn register_remote_action<A>(&mut self, name: &str, action: A)
        where
            A: ImplRemote + 'static,
    {
        self.cfb().register_remote_action(name, action);
    }
    /// setup the port for the server
    pub fn http_serv(&mut self, port: u16) {
        self.cfb().http_serv(port)
    }

    /// Tracer that will be used to save the tracing information.
    pub fn tracer(&mut self, tr: Tracer) {
        self.cfb().tracer(tr);
    }
    /// A file that has a snapshot of the bb in json format.
    pub fn bb_load(&mut self, bb: String) {
        self.cfb().bb_load(bb);
    }

    /// Mix the runtime async env(tokio env)
    /// By default, creates the default tokio Runtime multi thread
    pub fn rt_env(&mut self, env: RtEnv) {
        self.cfb().rt_env(env);
    }

    /// The method to build forester
    pub fn build(self) -> RtResult<Forester> {
        self.build_with(|| ActionImpl::Absent)
    }

    /// The method to build forester and provide the implementation for the absent actions
    pub fn build_with<T>(self, default_action: T) -> RtResult<Forester>
        where
            T: Fn() -> ActionImpl,
    {
        self.error()?;

        let (
            tree,
            actions,
            action_names,
            daemons,
            tr,
            env,
            bb_load,
            root,
            port
        ) = match self {
            ForesterBuilder::Files { delegate, cfb, .. } => {
                let root = delegate.root.clone();
                let project = delegate.build()?;
                let RuntimeTreeStarter {
                    tree,
                    std_actions,
                    actions,
                } = RuntimeTree::build(project)?;
                let mut impl_actions = cfb.actions;

                for (action_name, file_name) in std_actions.iter() {
                    impl_actions.insert(
                        action_name.clone(),
                        builtin::pick_action(action_name, file_name)?,
                    );
                }
                (
                    tree,
                    impl_actions,
                    actions,
                    cfb.daemons,
                    cfb.tracer,
                    cfb.env,
                    cfb.bb_load,
                    root,
                    cfb.port,
                )
            }
            ForesterBuilder::Text { delegate, cfb, .. } => {
                let project = delegate.build()?;
                let RuntimeTreeStarter {
                    tree,
                    std_actions,
                    actions,
                } = RuntimeTree::build(project)?;
                let mut impl_actions = cfb.actions;
                for (action_name, file_name) in std_actions.iter() {
                    let action = builtin::pick_action(action_name, file_name)?;
                    impl_actions.insert(action_name.clone(), action);
                }
                (
                    tree,
                    impl_actions,
                    actions,
                    cfb.daemons,
                    cfb.tracer,
                    cfb.env,
                    cfb.bb_load,
                    None,
                    cfb.port,
                )
            }
            ForesterBuilder::Code { delegate, cfb, .. } => {
                let (tree, actions) = delegate.build()?;
                (
                    tree,
                    cfb.actions,
                    actions,
                    cfb.daemons,
                    cfb.tracer,
                    cfb.env,
                    cfb.bb_load,
                    None,
                    cfb.port,
                )
            }
        };

        let bb =
            if let Some(bb_load_dump) = bb_load {
                BlackBoard::load(&get_pb(&PathBuf::from(bb_load_dump), &root)?)?
            } else { BlackBoard::default() };

        let mut env = if let Some(e) = env {
            e
        } else {
            RtEnv::try_new()?
        };


        let bb = Arc::new(Mutex::new(bb));
        let tracer = Arc::new(Mutex::new(tr));

        let context = DaemonContext::new(bb.clone(), tracer.clone());
        for daemon_cfg in daemons.into_iter() {
            match daemon_cfg {
                DaemonTaskCfg::Unnamed(d) => {
                    env.start_daemon(d, context.clone())?;
                }
                DaemonTaskCfg::Named(name, d) => {
                    env.start_named_daemon(name, d, context.clone())?;
                }
            }
        }

        let env = Arc::new(Mutex::new(env));
        let serv = if port.is_some() {
            Some(serv::start(env.clone(), port, bb.clone(), tracer.clone())?)
        } else {
            None
        };

        let keeper = ActionKeeper::new_with(actions, action_names, default_action)?;

        Forester::new(tree, bb, tracer, keeper, env, serv)
    }

    fn cfb(&mut self) -> &mut CommonForesterBuilder {
        match self {
            ForesterBuilder::Files { cfb, .. }
            | ForesterBuilder::Text { cfb, .. }
            | ForesterBuilder::Code { cfb, .. } => cfb,
        }
    }
    fn error(&self) -> RtOk {
        match &self {
            ForesterBuilder::Files { error: Some(v), .. }
            | ForesterBuilder::Text { error: Some(v), .. }
            | ForesterBuilder::Code { error: Some(v), .. } => {
                Err(RuntimeError::Unexpected(v.to_string()))
            }
            _ => Ok(()),
        }
    }
}

pub struct CommonForesterBuilder {
    env: Option<RtEnv>,
    tracer: Tracer,
    bb_load: Option<String>,
    actions: HashMap<ActionName, Action>,
    daemons: Vec<DaemonTaskCfg>,
    port: ServerPort,
}

impl CommonForesterBuilder {
    pub fn new() -> Self {
        Self {
            env: None,
            tracer: Tracer::noop(),
            bb_load: None,
            actions: HashMap::new(),
            daemons: Vec::new(),
            port: ServerPort::None,
        }
    }

    /// Add a daemon
    pub fn register_daemon(&mut self, daemon: Daemon) {
        self.daemons.push(DaemonTaskCfg::Unnamed(daemon));
    }

    /// Add a daemon with a name
    /// The name is used to stop the daemon
    pub fn register_named_daemon(&mut self, name: DaemonName, daemon: Daemon)
    {
        self.daemons.push(DaemonTaskCfg::Named(name, daemon));
    }


    /// Add an sync action according to the name.
    pub fn register_sync_action<A>(&mut self, name: &str, action: A)
        where
            A: Impl + 'static,
    {
        self.actions
            .insert(name.to_string(), Action::Sync(Box::new(action)));
    }
    /// Add an sync action according to the name.
    pub fn register_async_action<A>(&mut self, name: &str, action: A)
        where
            A: ImplAsync + 'static,
    {
        self.actions
            .insert(name.to_string(), Action::Async(Arc::new(action)));
    }
    /// Add an action according to the name but with a promise the action remote.
    pub fn register_remote_action<A>(&mut self, name: &str, action: A)
        where
            A: ImplRemote + 'static,
    {
        self.actions
            .insert(name.to_string(), Action::Remote(Box::new(action)));
    }
    /// setup the port for the server
    pub fn http_serv(&mut self, port: u16) {
        self.port = ServerPort::Static(port)
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
}

/// The struct defines the information of the server.
#[derive(Debug, Clone)]
pub enum ServerPort {
    None,
    Static(u16),
}

impl Default for ServerPort {
    fn default() -> Self {
        ServerPort::None
    }
}

impl ServerPort {
    fn is_some(&self) -> bool {
        match self {
            ServerPort::None => false,
            _ => true,
        }
    }
    fn get(&self) -> u16 {
        match self {
            ServerPort::None => 0,
            ServerPort::Static(v) => *v,
        }
    }
}

pub enum DaemonTaskCfg {
    Unnamed(Daemon),
    Named(DaemonName, Daemon),
}