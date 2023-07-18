use crate::runtime::env::RtEnv;
use crate::runtime::{RtResult, RuntimeError};
use crate::simulator::config::SimProfile;
use crate::simulator::Simulator;
use std::path::PathBuf;

/// The builder to create a simulator process.
///
/// # Examples
/// ```no-run
/// use std::path::PathBuf;
/// use forester::simulator::builder::SimulatorBuilder;
///
/// fn smoke() {
///     let mut sb = SimulatorBuilder::new();
///
///     let root = PathBuf("simulator/smoke");
///
///     sb.root(root);
///     sb.profile(PathBuf::from("sim.yaml"));
///     sb.main_file("main.tree".to_string());
///
///     let mut sim = sb.build().unwrap();
///     sim.run().unwrap();
/// }
///
/// fn smoke_from_text() {
///     let mut sb = SimulatorBuilder::new();
///
///     let sim = PathBuf("simulator/smoke/sim.yaml");
///
///     sb.profile(sim);
///     
///     sb.text(
///         r#"
/// import "std::actions"
///
/// root main sequence {
///     store_str("info1", "initial")
///     retryer(task(config = obj), success())
///     store_str("info2","finish")
/// }
///
/// fallback retryer(t:tree, default:tree){
///     retry(5) t(..)
///     fail("just should fail")
///     default(..)
/// }
///
/// impl task(config: object);
///     "#
///         .to_string(),
///     );    
///
///     let mut sim = sb.build().unwrap();
///     sim.run().unwrap();
/// }
/// ```
#[derive(Default)]
pub struct SimulatorBuilder {
    profile: Option<PathBuf>,
    root: Option<PathBuf>,
    main_file: Option<String>,
    main_tree: Option<String>,
    env: Option<RtEnv>,
    text: Option<String>,
}

impl SimulatorBuilder {
    pub fn new() -> Self {
        SimulatorBuilder {
            profile: None,
            root: None,
            main_file: None,
            main_tree: None,
            env: None,
            text: None,
        }
    }
    /// Add a simulator profile.
    pub fn profile(&mut self, profile: PathBuf) {
        self.profile = Some(profile);
    }
    /// Add a root folder.
    pub fn root(&mut self, root: PathBuf) {
        self.root = Some(root);
    }
    /// Add a main file.
    pub fn main_file(&mut self, main_file: String) {
        self.main_file = Some(main_file);
    }
    /// Add a main tree.
    pub fn main_tree(&mut self, main_tree: String) {
        self.main_tree = Some(main_tree);
    }

    /// Mix the runtime async env(tokio env)
    /// By default, creates the default tokio Runtime multi thread
    pub fn rt_env(&mut self, env: RtEnv) {
        self.env = Some(env);
    }

    /// add script on the fly.
    /// In that scenario, there is no need in the other attributes like files or root.
    ///
    /// Precautions:
    /// Imports and other files still work only with absolute paths.
    pub fn text(&mut self, text: String) {
        self.text = Some(text)
    }

    /// Build
    pub fn build(&mut self) -> RtResult<Simulator> {
        if let Some(root) = &self.root {
            let profile = match &self.profile {
                None => SimProfile::default(),
                Some(p) if p.is_relative() => {
                    let mut profile = root.clone();
                    profile.push(p);
                    SimProfile::parse_file(&profile)?
                }
                Some(p) => SimProfile::parse_file(&p)?,
            };

            if let Some(main_file) = &self.main_file {
                let env = if self.env.is_some() {
                    self.env.take().unwrap()
                } else {
                    RtEnv::try_new()?
                };
                Simulator::build(
                    profile,
                    root.clone(),
                    main_file.to_string(),
                    self.main_tree.clone(),
                    env,
                )
            } else {
                Err(RuntimeError::IOError(format!(
                    "the main file is not selected"
                )))
            }
        } else if let Some(text) = &self.text {
            let profile = match &self.profile {
                None => SimProfile::default(),
                Some(p) if p.is_relative() => {
                    return Err(RuntimeError::IOError(format!(
                        "the path to the given profile is relative but the root does not exist. Probably, the option from text has been selected."
                    )))
                }
                Some(p) => SimProfile::parse_file(&p)?,
            };
            let env = if self.env.is_some() {
                self.env.take().unwrap()
            } else {
                RtEnv::try_new()?
            };

            Simulator::build_from_text(profile, text.to_string(), env)
        } else {
            Err(RuntimeError::IOError(format!(
                "the root directory is not found"
            )))
        }
    }
}
