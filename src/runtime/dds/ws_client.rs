use serde::{Deserialize, Serialize};
use tungstenite::{connect, Message};
use url::Url;


#[derive(Serialize, Deserialize)]
pub struct PubMes<D: Serialize> {
    pub topic: String,
    pub msg: D,
}

impl<D: Serialize> PubMes<D> {
    pub fn new(topic: String, msg: D) -> Self {
        Self { topic, msg }
    }
}

pub fn publish<D: Serialize>(mes: PubMes<D>, url: String) {
    let (mut socket, response) =
        connect(Url::parse(url.as_str()).unwrap()).expect("Can't connect");
    println!("Connected to the server");
    println!("Response HTTP code: {}", response.status());
    println!("Response contains the following headers:");
    for (ref header, _value) in response.headers() {
        println!("* {}", header);
    }
    let js = serde_json::to_string_pretty(&mes).unwrap();
    socket.send(Message::text(js)).unwrap();
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use crate::runtime::args::RtValue;
    use crate::runtime::dds::ws_client::{publish, PubMes};

    #[test]
    fn smoke() {
        let value = RtValue::Object(HashMap::from_iter(vec![("a".to_string(), RtValue::int(10))]));
        publish(PubMes::new(
            "test".to_string(),
            value,
        ), "ws://localhost:9090".to_string());
    }
}



