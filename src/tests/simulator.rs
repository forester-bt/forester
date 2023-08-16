use crate::runtime::builder::ForesterBuilder;
use crate::simulator::builder::SimulatorBuilder;
use crate::simulator::config::{Action, BbConfig, SimProfile, SimProfileConfig, TracerSimConfig};
use crate::simulator::Simulator;
use crate::tests::test_folder;
use graphviz_rust::attributes::quadtree::fast;
use std::collections::HashMap;
use std::path::PathBuf;

#[test]
fn file() {
    let buf = test_folder("simulator/example.yaml");
    let profile = SimProfile::parse_file(&buf).unwrap();

    assert_eq!(
        SimProfile {
            config: SimProfileConfig {
                tracer: TracerSimConfig {
                    file: Some(String::from("main.trace")),
                    dt_fmt: None
                },
                graph: Some(String::from("main.svg")),
                max_ticks: Some(10),
                bb: BbConfig {
                    dump: Some(String::from("bb.dump")),
                    load: Some(String::from("bb.json")),
                },
                port: None,
            },
            actions: vec![
                Action {
                    name: "name".to_string(),
                    stub: "success".to_string(),
                    params: HashMap::from_iter(vec![("delay".to_string(), "10".to_string())]),
                },
                Action {
                    name: "name2".to_string(),
                    stub: "failure".to_string(),
                    params: HashMap::from_iter(vec![("delay".to_string(), "0".to_string())]),
                },
            ]
        },
        profile
    );

    let buf = test_folder("simulator/example2.yaml");
    let profile = SimProfile::parse_file(&buf).unwrap();

    assert_eq!(
        SimProfile {
            config: SimProfileConfig::default(),
            actions: vec![
                Action {
                    name: "name".to_string(),
                    stub: "success".to_string(),
                    params: HashMap::from_iter(vec![("delay".to_string(), "10".to_string())]),
                },
                Action {
                    name: "name2".to_string(),
                    stub: "failure".to_string(),
                    params: HashMap::from_iter(vec![("delay".to_string(), "0".to_string())]),
                },
            ]
        },
        profile
    );

    let buf = test_folder("simulator/example3.yaml");
    let profile = SimProfile::parse_file(&buf).unwrap();

    assert_eq!(
        SimProfile {
            config: SimProfileConfig::default(),
            actions: vec![Action {
                name: "name2".to_string(),
                stub: "failure".to_string(),
                params: HashMap::default(),
            },]
        },
        profile
    );

    let buf = test_folder("simulator/example4.yaml");
    let profile = SimProfile::parse_file(&buf).unwrap();

    assert_eq!(
        SimProfile {
            config: SimProfileConfig::default(),
            actions: vec![]
        },
        profile
    );
}

#[test]
fn smoke() {
    let mut sb = SimulatorBuilder::new();

    let root = test_folder("simulator/smoke");

    sb.root(root.clone());
    sb.profile(PathBuf::from("sim.yaml"));

    let mut fb = ForesterBuilder::from_fs();
    fb.main_file("main.tree".to_string());
    fb.root(root);

    sb.forester_builder(fb);

    let mut sim = sb.build().unwrap();
    let tracer = &sim.forester.tracer;
    sim.run().unwrap();
}

// #[test]
fn text() {
    let mut fb = ForesterBuilder::from_text();
    let mut sb = SimulatorBuilder::new();
    fb.text(
        r#"
import "std::actions"

root main sequence {
    store_str("info1", "initial")
    retryer(task(config = obj), success())
    store_str("info2","finish")
}

fallback retryer(t:tree, default:tree){
    retry(5) t(..)
    fail("just should fail")
    default(..)
}

impl task(config: object);
    "#
        .to_string(),
    );
    sb.forester_builder(fb);
    let sim = test_folder("simulator/smoke/sim_absolute.yaml");

    sb.profile(sim);

    let mut sim = sb.build().unwrap();
    sim.run().unwrap();
}
