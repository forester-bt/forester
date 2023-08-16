use crate::runtime::args::{RtValue, RtValueNumber};
use serde::de::{Error, MapAccess, SeqAccess, Visitor};

use serde::ser::{Error as seError, SerializeMap, SerializeSeq};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::HashMap;
use std::fmt::Formatter;

impl<'de> Deserialize<'de> for RtValue {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_any(RtValueVisitor)
    }
}
impl Serialize for RtValue {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        match self {
            RtValue::String(s) => serializer.serialize_str(s.as_ref()),
            RtValue::Bool(b) => serializer.serialize_bool(*b),
            RtValue::Number(n) => match n {
                RtValueNumber::Int(i) => serializer.serialize_i64(*i),
                RtValueNumber::Float(f) => serializer.serialize_f64(*f),
                RtValueNumber::Hex(i) => serializer.serialize_i64(*i),
                RtValueNumber::Binary(b) => serializer.serialize_i64(*b as i64),
            },
            RtValue::Array(elems) => {
                let mut seq = serializer.serialize_seq(Some(elems.len()))?;
                for element in elems {
                    seq.serialize_element(element)?;
                }
                seq.end()
            }
            RtValue::Object(map) => {
                let mut ser = serializer.serialize_map(Some(map.len()))?;
                for (k, v) in map {
                    ser.serialize_entry(k, v)?;
                }
                ser.end()
            }
            RtValue::Pointer(_) => Err(S::Error::custom("pointer can not be serialized")),
            RtValue::Call(_) => Err(S::Error::custom("call can not be serialized")),
        }
    }
}

struct RtValueVisitor;
impl<'de> Visitor<'de> for RtValueVisitor {
    type Value = RtValue;

    fn expecting(&self, formatter: &mut Formatter) -> std::fmt::Result {
        write!(formatter, "start deserialization")
    }

    fn visit_bool<E>(self, v: bool) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::Bool(v))
    }

    fn visit_i8<E>(self, v: i8) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::int(v as i64))
    }

    fn visit_i16<E>(self, v: i16) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::int(v as i64))
    }

    fn visit_i32<E>(self, v: i32) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::int(v as i64))
    }

    fn visit_i64<E>(self, v: i64) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::int(v))
    }

    fn visit_u8<E>(self, v: u8) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::int(v as i64))
    }

    fn visit_u16<E>(self, v: u16) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::int(v as i64))
    }

    fn visit_u32<E>(self, v: u32) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::int(v as i64))
    }

    fn visit_u64<E>(self, v: u64) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::int(v as i64))
    }

    fn visit_f32<E>(self, v: f32) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::Number(RtValueNumber::Float(v as f64)))
    }

    fn visit_f64<E>(self, v: f64) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::Number(RtValueNumber::Float(v)))
    }

    fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::String(v.to_string()))
    }

    fn visit_borrowed_str<E>(self, v: &'de str) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::String(v.to_string()))
    }

    fn visit_string<E>(self, v: String) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(RtValue::String(v))
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let mut v = vec![];
        while let Some(e) = seq.next_element::<RtValue>()? {
            v.push(e);
        }
        Ok(RtValue::Array(v))
    }

    fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
    where
        A: MapAccess<'de>,
    {
        let mut m = HashMap::new();
        while let Some((k, v)) = map.next_entry::<String, RtValue>()? {
            m.insert(k, v);
        }

        Ok(RtValue::Object(m))
    }
}
