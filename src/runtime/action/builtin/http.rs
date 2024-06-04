use crate::runtime::action::{Impl, ImplAsync, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::TreeContextRef;
use crate::runtime::{RuntimeError, TickResult};
/// Synchronous http get.
/// It accepts url for request and key in bb to write the results as string.
pub struct HttpGet;

impl HttpGet {
    fn on_tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let url = args
            .find_or_ith("url".to_string(), 0)
            .ok_or(RuntimeError::fail(
                "url is not found or it is not a string".to_string(),
            ))
            .and_then(|v| v.cast(ctx.clone()).str())?
            .ok_or(RuntimeError::fail(
                "url is not found or it is not a string".to_string(),
            ))?;

        let out = args
            .find_or_ith("bb_key".to_string(), 1)
            .ok_or(RuntimeError::fail(
                "bb_key is not found or it is not a string".to_string(),
            ))
            .and_then(|v| v.cast(ctx.clone()).str())?
            .ok_or(RuntimeError::fail(
                "bb_key is not found or it is not a string".to_string(),
            ))?;

        match reqwest::blocking::get(url).and_then(|v| v.text()) {
            Ok(resp) => {
                ctx.bb().lock()?.put(out, RtValue::str(resp))?;
                Ok(TickResult::success())
            }
            Err(err) => Ok(TickResult::failure(format!("error {}", err))),
        }
    }
}

impl Impl for HttpGet {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        self.on_tick(args, ctx)
    }
}

impl ImplAsync for HttpGet {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        self.on_tick(args, ctx)
    }
}

#[cfg(test)]
mod tests {
    use crate::runtime::action::builtin::http::HttpGet;
    use crate::runtime::action::Impl;
    use crate::runtime::args::{RtArgs, RtArgument, RtValue};
    use crate::runtime::blackboard::BlackBoard;
    use crate::runtime::context::TreeContextRef;
    use crate::runtime::env::RtEnv;
    use crate::runtime::trimmer::TrimmingQueue;
    use crate::runtime::TickResult;
    use crate::tracer::Tracer;
    
    use std::sync::{Arc, Mutex};
    use wiremock::matchers::{method, path};
    use wiremock::{Mock, MockServer, ResponseTemplate};

    #[test]
    fn smoke() {
        let env = RtEnv::try_new().unwrap();

        let port = env.runtime.block_on(async {
            let mock_server = MockServer::start().await;

            let resp = ResponseTemplate::new(200);
            let resp = resp.set_body_string("OK");

            Mock::given(method("GET"))
                .and(path("/hello"))
                .respond_with(resp)
                .mount(&mock_server)
                .await;
            mock_server.address().port()
        });

        let action = HttpGet;

        let bb = Arc::new(Mutex::new(BlackBoard::default()));
        let r = action.tick(
            RtArgs(vec![
                RtArgument::new_noname(RtValue::str(format!("http://localhost:{port}/hello"))),
                RtArgument::new_noname(RtValue::str("k".to_string())),
            ]),
            TreeContextRef::new(
                bb.clone(),
                Arc::new(Mutex::new(Tracer::Noop)),
                1,
                Arc::new(Mutex::new(TrimmingQueue::default())),
                Arc::new(Mutex::new(RtEnv::try_new().unwrap()))
            ),
        );

        assert_eq!(r, Ok(TickResult::success()));

        assert_eq!(
            bb.lock().unwrap().get("k".to_string()),
            Ok(Some(&RtValue::str("OK".to_string())))
        );
    }
}
