use std::net::TcpStream;
use serde::{Deserialize, Serialize, Serializer};
use serde::ser::SerializeStruct;
use tungstenite::{connect, Message, WebSocket};
use tungstenite::stream::MaybeTlsStream;
use url::Url;
use crate::runtime::action::Tick;
use crate::runtime::args::RtValue;
use crate::runtime::{RtResult, RuntimeError, TickResult};

type Topic = String;
type Type = String;

pub type WS = WebSocket<MaybeTlsStream<TcpStream>>;

#[derive(Debug, Default, Clone, Deserialize)]
pub struct SubscribeCfg {
    tp: Option<String>,
    throttle_rate: Option<i32>,
    queue_length: Option<i32>,
    fragment_size: Option<i32>,
    compression: Option<String>,
}


impl SubscribeCfg {
    pub fn from(v: RtValue) -> RtResult<SubscribeCfg> {
        let mut cfg = SubscribeCfg::default();
        if let RtValue::Object(map) = v {
            for (k, v) in map {
                match k.as_str() {
                    "tp" => {
                        cfg.tp = Some(v.as_string().ok_or(RuntimeError::fail("the type is not string".to_string()))?);
                    }
                    "throttle_rate" => {
                        cfg.throttle_rate = Some(v.as_int().map(|v| v as i32)
                            .ok_or(RuntimeError::fail("the throttle_rate is not int".to_string()))?);
                    }
                    "queue_length" => {
                        cfg.queue_length = Some(v.as_int().map(|v| v as i32)
                            .ok_or(RuntimeError::fail("the queue_length is not int".to_string()))?);
                    }
                    "fragment_size" => {
                        cfg.fragment_size = Some(v.as_int().map(|v| v as i32)
                            .ok_or(RuntimeError::fail("the fragment_size is not int".to_string()))?);
                    }
                    "compression" => {
                        cfg.compression = Some(v.as_string()
                            .ok_or(RuntimeError::fail("the compression is not string".to_string()))?);
                    }
                    _ => {
                        return Err(RuntimeError::fail(format!("the key {} is not supported", k)));
                    }
                }
            }
        } else {
            return Err(RuntimeError::fail("the source_cfg is not object".to_owned()));
        }

        Ok(cfg)
    }
    pub fn count(&self) -> usize {
        let mut count = 0;

        if self.tp.is_some() {
            count += 1;
        }
        if self.throttle_rate.is_some() {
            count += 1;
        }
        if self.queue_length.is_some() {
            count += 1;
        }
        if self.fragment_size.is_some() {
            count += 1;
        }
        if self.compression.is_some() {
            count += 1;
        }


        count
    }
    pub fn new(tp: Option<String>, throttle_rate: Option<i32>, queue_length: Option<i32>, fragment_size: Option<i32>, compression: Option<String>) -> Self {
        Self { tp, throttle_rate, queue_length, fragment_size, compression }
    }
}

#[derive(Deserialize)]
pub enum RosCommand
{
    Publish(Topic, RtValue),
    Advertise(Topic, Type),
    Unsubscribe(Topic),
    Subscribe(Topic, SubscribeCfg),

}

impl Serialize for RosCommand {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
        match self {
            RosCommand::Publish(t, v) => {
                let mut cmd = serializer.serialize_struct("Command", 3)?;
                cmd.serialize_field("op", "publish")?;
                cmd.serialize_field("topic", t)?;
                cmd.serialize_field("msg", v)?;
                cmd.end()
            }
            RosCommand::Advertise(t, tp) => {
                let mut cmd = serializer.serialize_struct("Command", 3)?;
                cmd.serialize_field("op", "advertise")?;
                cmd.serialize_field("topic", t)?;
                cmd.serialize_field("type", tp)?;
                cmd.end()
            }
            RosCommand::Unsubscribe(t) => {
                let mut cmd = serializer.serialize_struct("Command", 2)?;
                cmd.serialize_field("op", "unsubscribe")?;
                cmd.serialize_field("topic", t)?;
                cmd.end()
            }
            RosCommand::Subscribe(_t, cfg) => {
                let cfg_count = cfg.count();
                let mut cmd = serializer.serialize_struct("Command", cfg_count + 1)?;
                cmd.serialize_field("op", "subscribe")?;

                if let Some(tp) = &cfg.tp {
                    cmd.serialize_field("type", tp)?;
                }
                if let Some(throttle_rate) = &cfg.throttle_rate {
                    cmd.serialize_field("throttle_rate", throttle_rate)?;
                }
                if let Some(queue_length) = &cfg.queue_length {
                    cmd.serialize_field("queue_length", queue_length)?;
                }
                if let Some(fragment_size) = &cfg.fragment_size {
                    cmd.serialize_field("fragment_size", fragment_size)?;
                }
                if let Some(compression) = &cfg.compression {
                    cmd.serialize_field("compression", compression)?;
                }

                cmd.end()
            }
        }
    }
}

impl RosCommand {
    pub fn publish(topic: Topic, mes: RtValue) -> RosCommand {
        RosCommand::Publish(topic, mes)
    }
    pub fn advertise(topic: Topic, tp: Type) -> RosCommand {
        RosCommand::Advertise(topic, tp)
    }
    pub fn unsubscribe(topic: Topic) -> RosCommand {
        RosCommand::Unsubscribe(topic)
    }
    pub fn subscribe(topic: Topic, cfg: SubscribeCfg) -> RosCommand {
        RosCommand::Subscribe(topic, cfg)
    }
}


pub fn publish(topic: Topic, mes: RtValue, url: String) -> Tick {
    debug!(target: "ws-publish" ,"params: topic: {topic}, mes: {mes}, url: {url}");
    let (mut socket, response) =
        connect(Url::parse(url.as_str())?)?;
    debug!(target: "ws" ,"Connected to the server {url}");
    debug!(target: "ws" ,"Response HTTP code: {}", response.status());
    let js = serde_json::to_string_pretty(&RosCommand::publish(topic, mes))?;
    socket.send(Message::text(js))?;

    Ok(TickResult::success())
}

pub fn advertise(topic: Topic, tp: Type, url: String) -> Tick {
    debug!(target: "ws-advertise" ,"params: topic: {topic}, type: {tp}, url: {url}");
    let (mut socket, response) =
        connect(Url::parse(url.as_str()).unwrap())?;
    debug!(target: "ws" ,"Connected to the server {url}");
    debug!(target: "ws" ,"Response HTTP code: {}", response.status());
    let js = serde_json::to_string_pretty(&RosCommand::advertise(topic, tp))?;
    socket.send(Message::text(js))?;

    Ok(TickResult::success())
}

pub fn unsubscribe(topic: Topic, url: String) -> Tick {
    debug!(target: "ws-unsubscribe" ,"params: topic: {topic}, url: {url}");

    // send the response to unsubscribe
    let (mut socket, response) =
        connect(Url::parse(url.as_str()).unwrap())?;
    debug!(target: "ws" ,"Connected to the server {url}");
    debug!(target: "ws" ,"Response HTTP code: {}", response.status());
    let js = serde_json::to_string_pretty(&RosCommand::unsubscribe(topic))?;
    socket.send(Message::text(js))?;

    Ok(TickResult::success())
}

pub fn subscribe(topic: Topic, cfg: SubscribeCfg, url: String) -> RtResult<WS> {
    debug!(target: "ws-subscribe" ,"params: topic: {topic}, cfg:{:?} url: {url}",cfg);
    let (mut socket, response) =
        connect(Url::parse(url.as_str()).unwrap())?;
    debug!(target: "ws" ,"Connected to the server {url}");
    debug!(target: "ws" ,"Response HTTP code: {}", response.status());
    let js = serde_json::to_string_pretty(&RosCommand::subscribe(topic, cfg))?;
    socket.send(Message::text(js))?;

    Ok(socket)
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use crate::runtime::args::RtValue;
    use crate::runtime::ros::client::{publish};

    #[test]
    fn smoke() {
        let value = RtValue::Object(HashMap::from_iter(vec![("a".to_string(), RtValue::int(10))]));
        publish("test".to_owned(), value, "ws://localhost:9090".to_string())
            .expect("TODO: panic message");
    }
}



