use serde::{Deserialize, Serialize, Serializer};
use serde::ser::SerializeStruct;
use tungstenite::{connect, Message};
use url::Url;
use crate::runtime::action::Tick;
use crate::runtime::args::RtValue;
use crate::runtime::TickResult;

type Topic = String;
type Type = String;

#[derive(Deserialize)]
pub enum RosCommand
{
    Publish(Topic, RtValue),
    Advertise(Topic, Type),

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
        }
    }
}

impl RosCommand {
    pub fn publish(topic: Topic, mes: RtValue) -> RosCommand {
        RosCommand::Publish(topic, mes)
    }
}


pub fn publish(mes: RosCommand, url: String) -> Tick {
    let (mut socket, response) =
        connect(Url::parse(url.as_str()).unwrap())?;
    debug!(target: "ws" ,"Connected to the server {url}");
    debug!(target: "ws" ,"Response HTTP code: {}", response.status());
    let js = serde_json::to_string_pretty(&mes)?;
    socket.send(Message::text(js))?;

    Ok(TickResult::success())
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use crate::runtime::args::RtValue;
    use crate::runtime::ros::ws_client::{publish, RosCommand};

    #[test]
    fn smoke() {
        let value = RtValue::Object(HashMap::from_iter(vec![("a".to_string(), RtValue::int(10))]));
        publish(RosCommand::publish("test".to_owned(),value), "ws://localhost:9090".to_string())
            .expect("TODO: panic message");
    }
}



