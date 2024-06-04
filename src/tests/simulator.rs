use crate::runtime::action::builtin::remote::RemoteActionRequest;
use crate::runtime::builder::ForesterBuilder;
use crate::runtime::env::RtEnv;
use crate::runtime::TickResult;
use crate::simulator::builder::SimulatorBuilder;
use crate::simulator::config::{
    Action, BbConfig, HttpServ, SimProfile, SimProfileConfig, TracerSimConfig,
};

use crate::tests::{test_folder, turn_on_logs};
use axum::http::StatusCode;
use axum::response::IntoResponse;
use axum::routing::{get, post};
use axum::{Json, Router, ServiceExt};
use forester_http::client::ForesterHttpClient;

use serde_json::json;
use std::collections::HashMap;
use std::io::Write;
use std::net::SocketAddr;
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
                http: None,
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

    let buf = test_folder("simulator/example5.yaml");
    let profile = SimProfile::parse_file(&buf).unwrap();

    assert_eq!(
        SimProfile {
            config: SimProfileConfig {
                tracer: Default::default(),
                bb: Default::default(),
                graph: None,
                max_ticks: None,
                http: Some(HttpServ { port: 8080 }),
            },
            actions: vec![
                Action {
                    name: "name1".to_string(),
                    stub: "remote".to_string(),
                    params: HashMap::from_iter(vec![
                        ("url".to_string(), "localhost:10000".to_string()),
                        ("server".to_string(), "http://localhost:8080".to_string()),
                    ])
                },
                Action {
                    name: "name2".to_string(),
                    stub: "remote".to_string(),
                    params: HashMap::from_iter(vec![(
                        "url".to_string(),
                        "localhost:10001".to_string()
                    ),])
                }
            ]
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
    let result = sim.run().unwrap();
    assert_eq!(result, TickResult::Success);
}

#[ignore]
#[test]
fn smoke_remote() {
    let env = RtEnv::try_new().unwrap();
    let _ = env.runtime.spawn_blocking(|| async {
        async fn handler(Json(req): Json<RemoteActionRequest>) -> impl IntoResponse {
            let url = req.clone().serv_url;

            let client = ForesterHttpClient::new(url);
            let _ = client.put("tt".to_string(), json!("OK")).await;

            (StatusCode::OK, Json::from(TickResult::Success))
        }

        let routing = Router::new()
            .route("/", get(|| async { "OK" }))
            .route("/action", post(handler))
            .into_make_service();

        axum::Server::bind(&SocketAddr::from(([127, 0, 0, 1], 10000)))
            .serve(routing)
            .await
            .unwrap();
    });

    turn_on_logs();
    let mut sb = SimulatorBuilder::new();

    let root = test_folder("simulator/smoke_rem");

    sb.root(root.clone());
    sb.profile(PathBuf::from("sim.yaml"));

    let mut fb = ForesterBuilder::from_fs();
    fb.rt_env(env);
    fb.main_file("main.tree".to_string());
    fb.root(root);

    sb.forester_builder(fb);

    let mut sim = sb.build().unwrap();
    sim.run().unwrap();

    let bb = sim.forester.bb.lock().unwrap();
    let option = bb.get("tt".to_string()).unwrap();
    assert_eq!(
        option.and_then(|v| v.clone().as_string()),
        Some("OK".to_string())
    );
}

#[test]
fn text() {
    test_folder("simulator/smoke");

    let mut fb = ForesterBuilder::from_text();
    let mut sb = SimulatorBuilder::new();
    fb.text(
        r#"
import "std::actions"

root main sequence {
    store("info1", "initial")
    retryer(task(config = obj), success())
    store("info2","finish")
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

    let buf = test_folder("simulator/smoke/gen");
    let gen = buf.as_path().to_str().unwrap();

    let sim = test_folder("simulator/smoke/sim_absolute.yaml"); // <_Absolute_path_>
    let new_sim_profile = test_folder("simulator/smoke/sim_absolute_t.yaml"); // <_Absolute_path_>
    let sim_abs = std::fs::read_to_string(sim)
        .unwrap()
        .replace("<_Absolute_path_>", gen);
    let mut file = std::fs::File::create(new_sim_profile.clone()).unwrap();
    write!(file, "{}", sim_abs).unwrap();

    sb.profile(new_sim_profile);

    let mut sim = sb.build().unwrap();
    let result = sim.run().unwrap();
    assert_eq!(result, TickResult::Success);
}
