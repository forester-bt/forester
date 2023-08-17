use crate::runtime::args::RtValue;
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::builder::ServerPort;
use crate::runtime::env::RtEnv;
use crate::runtime::forester::serv::{start, HttpServ};
use crate::tests::turn_on_logs;
use crate::tracer::Tracer;
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[test]
fn smoke_stop() {
    turn_on_logs();
    let bb = Arc::new(Mutex::new(BlackBoard::default()));
    let tr = Arc::new(Mutex::new(Tracer::default()));

    let rt = RtEnv::try_new().unwrap().runtime;

    let info = start(&rt, ServerPort::Static(10000), bb.clone(), tr.clone()).unwrap();
    let stop = info.stop_cmd;

    rt.spawn(async {
        tokio::time::sleep(Duration::from_secs(2)).await;
        // stop.send(()).unwrap();
    });

    rt.block_on(async {
        info.status.await.unwrap().unwrap();
    })
}
