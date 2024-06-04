use crate::runtime::action::builtin::remote::RemoteHttpAction;
use crate::runtime::action::{Impl, ImplRemote};
use crate::runtime::args::{RtArgs, RtArgument, RtValue};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::builder::ServerPort;
use crate::runtime::context::{TreeRemoteContextRef};
use crate::runtime::env::RtEnv;
use crate::runtime::forester::serv::start;
use crate::runtime::TickResult;
use crate::tests::{fb, turn_on_logs};
use crate::tracer::Tracer;
use serde_json::json;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use wiremock::matchers::{method, path};
use wiremock::{Mock, MockServer, ResponseTemplate};

#[test]
fn smoke_serv() {
    turn_on_logs();
    let bb = Arc::new(Mutex::new(BlackBoard::default()));
    let tr = Arc::new(Mutex::new(Tracer::default()));

    let rt = Arc::new(Mutex::new(RtEnv::try_new().unwrap()));

    let info = start(rt.clone(), ServerPort::Static(20000), bb.clone(), tr.clone()).unwrap();
    let stop = info.stop_cmd;

    let runtime =  rt.lock().unwrap();
    runtime.runtime.spawn(async {
        tokio::time::sleep(Duration::from_secs(1)).await;
        stop.send(()).unwrap();
    });

    runtime.runtime.block_on(async {
        info.status.await.unwrap().unwrap();
    })
}

#[test]
fn remote_smoke() {
    turn_on_logs();
    let env = RtEnv::try_new().unwrap();

    let port = env.runtime.block_on(async {
        let mock_server = MockServer::start().await;

        let resp = ResponseTemplate::new(200);
        let resp = resp.set_body_json(json!("Success"));

        Mock::given(method("POST"))
            .and(path("/action"))
            .respond_with(resp)
            .mount(&mock_server)
            .await;
        mock_server.address().port()
    });

    let action = RemoteHttpAction::new(format!("http://localhost:{}/action", port));

    let result = action.tick(
        RtArgs(vec![
            RtArgument::new("a".to_string(), RtValue::int(1)),
            RtArgument::new("b".to_string(), RtValue::str("a".to_string())),
            RtArgument::new(
                "c".to_string(),
                RtValue::Array(vec![RtValue::int(1), RtValue::int(2)]),
            ),
        ]),
        TreeRemoteContextRef::new(1, 9999, Arc::new(Mutex::new(env))),
    );

    assert_eq!(result, Ok(TickResult::success()));
}

#[test]
fn remote_in_tree() {
    turn_on_logs();
    let env = RtEnv::try_new().unwrap();
    let port = env.runtime.block_on(async {
        let mock_server = MockServer::start().await;

        let resp = ResponseTemplate::new(200);
        let resp = resp.set_body_json(json!("Success"));

        Mock::given(method("POST"))
            .and(path("/action"))
            .respond_with(resp)
            .mount(&mock_server)
            .await;
        mock_server.address().port()
    });

    let mut builder = fb("actions/remote");
    builder.rt_env(env);
    builder.tracer(Tracer::default());

    let action = RemoteHttpAction::new(format!("http://localhost:{}/action", port));
    builder.register_remote_action("action", action);
    builder.http_serv(9999);
    let mut f = builder.build().unwrap();

    let result = f.run();

    assert_eq!(result, Ok(TickResult::success()));
}
