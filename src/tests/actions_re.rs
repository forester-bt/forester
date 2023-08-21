use crate::runtime::action::builtin::remote::RemoteHttpAction;
use crate::runtime::action::{Impl, ImplRemote};
use crate::runtime::args::{RtArgs, RtArgument, RtValue};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::builder::ServerPort;
use crate::runtime::context::{TreeContextRef, TreeRemoteContextRef};
use crate::runtime::env::RtEnv;
use crate::runtime::forester::serv::start;
use crate::tests::{fb, turn_on_logs};
use crate::tracer::Tracer;
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[test]
fn smoke_serv() {
    turn_on_logs();
    let bb = Arc::new(Mutex::new(BlackBoard::default()));
    let tr = Arc::new(Mutex::new(Tracer::default()));

    let rt = RtEnv::try_new().unwrap().runtime;

    let info = start(&rt, ServerPort::Static(9999), bb.clone(), tr.clone()).unwrap();
    let stop = info.stop_cmd;

    rt.spawn(async {
        tokio::time::sleep(Duration::from_secs(2)).await;
        // stop.send(()).unwrap();
    });

    rt.block_on(async {
        info.status.await.unwrap().unwrap();
    })
}

#[test]
fn remote_smoke() {
    turn_on_logs();
    let action = RemoteHttpAction::new("http://localhost:10000/action".to_string());
    let mut env = RtEnv::try_new().unwrap();
    let result = action.tick(
        RtArgs(vec![
            RtArgument::new("a".to_string(), RtValue::int(1)),
            RtArgument::new("b".to_string(), RtValue::str("a".to_string())),
            RtArgument::new(
                "c".to_string(),
                RtValue::Array(vec![RtValue::int(1), RtValue::int(2)]),
            ),
        ]),
        TreeRemoteContextRef::new(1, 9999, &mut env),
    );

    println!("{:?}", result);
    assert_eq!(1, 0)
}
