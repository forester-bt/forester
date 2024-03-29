use crate::runtime::action::Action as RtAction;
use crate::runtime::action::Tick;
use crate::runtime::forester::Forester;

use crate::simulator::config::SimProfile;

use crate::visualizer::Visualizer;
use std::path::PathBuf;
pub mod actions;
pub mod builder;
pub mod config;
use crate::get_pb;

/// Simulator is a  wrapper above Forester that moderates a couple of extra things.
/// The idea is to stub some actions and to get a control run of the tree
/// under the different circumstances and initial conditions.
/// Therefore it can be done without codding the stubs, just using the given stubs with different profiles.
///
/// # Example
/// # Example
/// ```no_run
/// use std::path::PathBuf;
/// use forester_rs::runtime::builder::ForesterBuilder;
/// use forester_rs::simulator::builder::SimulatorBuilder;
///
/// fn smoke() {
///     let mut sb = SimulatorBuilder::new();
///
///     let root = PathBuf::from("simulator/smoke");
///
///     sb.root(root.clone());
///     sb.profile(PathBuf::from("sim.yaml"));
///     
///     let mut fb = ForesterBuilder::from_fs();
///     
///     fb.root(root);    
///
///     fb.main_file("main.tree".to_string());
///     
///     sb.forester_builder(fb);    
///
///     let mut sim = sb.build().unwrap();
///     sim.run().unwrap();
/// }
/// ```
pub struct Simulator {
    pub root: Option<PathBuf>,
    pub profile: SimProfile,
    pub forester: Forester,
}

impl Simulator {
    pub fn run(&mut self) -> Tick {
        let cfg = &self.profile.config;
        let max = cfg.max_ticks;

        if let Some(viz_file) = &cfg.graph {
            let tree = &self.forester.tree;
            Visualizer::rt_tree_svg_to_file(tree, get_pb(&PathBuf::from(viz_file), &self.root)?)?;
        }

        let result = self.forester.run_until(max);

        if let Some(bb_dump) = &cfg.bb.dump {
            self.forester
                .bb
                .lock()?
                .dump(get_pb(&PathBuf::from(bb_dump), &self.root)?)?;
        }

        result
    }

    pub fn new(root: Option<PathBuf>, profile: SimProfile, forester: Forester) -> Self {
        Self {
            root,
            profile,
            forester,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::runtime::builder::ForesterBuilder;
    use crate::runtime::TickResult;
    use crate::simulator::builder::SimulatorBuilder;

    #[test]
    fn smoke_all_success() {
        let mut sb = SimulatorBuilder::new();
        let mut fb = ForesterBuilder::from_text();
        fb.text(
            r#"
         
         impl action1();   
         impl action2();   
            
        root main sequence {
                action1()
                action2()
        }
        "#
            .to_string(),
        );
        sb.forester_builder(fb);

        let mut simulator = sb.build().unwrap();
        let result = simulator.run().unwrap();

        assert_eq!(result, TickResult::Success);
    }
}
