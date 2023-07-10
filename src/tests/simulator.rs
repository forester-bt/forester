use crate::simulator::builder::SimulatorBuilder;
use crate::simulator::config::{Action, BbConfig, SimProfile, SimProfileConfig};
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
                trace: Some(String::from("main.trace")),
                graph: Some(String::from("main.svg")),
                max_ticks: Some(10),
                bb: BbConfig {
                    dump: Some(String::from("bb.dump")),
                    load: Some(String::from("bb.json")),
                },
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

    sb.root(root);
    sb.profile(PathBuf::from("sim.yaml"));
    sb.main_file("main.tree".to_string());

    let mut sim = sb.build().unwrap();
    let tracer = &sim.forester.tracer;
    sim.run().unwrap();
}
