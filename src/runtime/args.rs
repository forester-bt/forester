pub mod display;
mod sede;
pub mod transform;

use crate::runtime::args::transform::find_arg_value;
use crate::runtime::blackboard::BBKey;
use crate::runtime::context::TreeContextRef;
use crate::runtime::{RtResult, RuntimeError};
use crate::tree::parser::ast::arg::{ArgumentRhs, Arguments, MesType, Param, Params};
use crate::tree::parser::ast::call::Call;
use crate::tree::parser::ast::message::{Message, Number};
use crate::tree::{cerr, TreeError};
use itertools::Itertools;

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt::{Display, Formatter};

/// Just a Key class for the arguments that represents the key in BB
pub type RtAKey = String;

/// The structure that represents the number type in runtime.
/// # Notes
/// The number can be represented in different formats:
/// - `Int` - the integer number
/// - `Float` - the floating point number
/// - `Hex` - the hexadecimal number
/// - `Binary` - the binary number
#[derive(Debug, PartialEq, Clone, Deserialize, Serialize)]
pub enum RtValueNumber {
    Int(i64),
    Float(f64),
    Hex(i64),
    Binary(isize),
}

impl From<Number> for RtValueNumber {
    fn from(value: Number) -> Self {
        match value {
            Number::Int(i) => RtValueNumber::Int(i),
            Number::Float(f) => RtValueNumber::Float(f),
            Number::Hex(h) => RtValueNumber::Hex(h),
            Number::Binary(b) => RtValueNumber::Binary(b),
        }
    }
}

/// The structure that represents the value type in runtime.
/// The value type is a value in BlackBoard or a message.
/// # Notes
/// The value can be represented in different formats:
/// - `String` - the string value
/// - `Bool` - the boolean value
/// - `Array` - the array of values
/// - `Object` - the object of values
/// - `Number` - the number value
/// - `Pointer` - the pointer to the value in BlackBoard (or to parent arguments)
/// - `Call` - the call to the tree (for the higher order trees)
#[derive(Debug, PartialEq, Clone)]
pub enum RtValue {
    String(String),
    Bool(bool),
    Array(Vec<RtValue>),
    Object(HashMap<String, RtValue>),
    Number(RtValueNumber),
    Pointer(BBKey),
    Call(Call),
}

/// Just a utility helping to cast the `RtValue` to the specific type.
/// ```rust
///     use forester_rs::runtime::args::RtArgs;
///     use forester_rs::runtime::context::{TreeContext, TreeContextRef};
///  fn tick(args: RtArgs, ctx: TreeContextRef) {
///      match args.first() {
///         None => (),
///         Some(v) => {
///             let val = v.cast(ctx.clone()).str().unwrap().unwrap_or_default();
///             println!("{val}");
///         }
///     }
/// }
///
/// ```
pub struct RtValueCast {
    v: RtValue,
    ctx: TreeContextRef,
}

impl RtValueCast {
    /// tries to convert the value to the given type but
    /// considers the value as a pointer to the value in BlackBoard
    pub fn with_ptr(self) -> RtResult<RtValue> {
        self.v.with_ptr(self.ctx)
    }

    pub fn str(self) -> RtResult<Option<String>> {
        self.with_ptr().map(RtValue::as_string)
    }
    pub fn int(self) -> RtResult<Option<i64>> {
        self.with_ptr().map(RtValue::as_int)
    }
    pub fn bool(self) -> RtResult<Option<bool>> {
        self.with_ptr().map(RtValue::as_bool)
    }
    pub fn float(self) -> RtResult<Option<f64>> {
        self.with_ptr().map(RtValue::as_float)
    }
    /// tries to convert to vec and map each element
    pub fn map_vec<Map, To>(self, map: Map) -> RtResult<Option<Vec<To>>>
    where
        Map: Fn(RtValue) -> To,
    {
        self.with_ptr().map(|v| v.as_vec(map))
    }

    /// tries to convert obj to map
    pub fn map_obj<Map, To>(self, map: Map) -> RtResult<Option<HashMap<String, To>>>
    where
        Map: Fn((String, RtValue)) -> (String, To),
    {
        self.with_ptr().map(|v| v.as_map(map))
    }
}

impl RtValue {
    pub fn int(i: i64) -> Self {
        RtValue::Number(RtValueNumber::Int(i))
    }
    pub fn float(f: f64) -> Self {
        RtValue::Number(RtValueNumber::Float(f))
    }
    pub fn str(s: String) -> Self {
        RtValue::String(s)
    }
    /// cast to the given type with the consideration of the pointers
    pub fn cast(self, ctx: TreeContextRef) -> RtValueCast {
        RtValueCast { v: self, ctx }
    }

    pub fn as_string(self) -> Option<String> {
        match self {
            RtValue::String(v) => Some(v),
            _ => None,
        }
    }
    pub fn as_bool(self) -> Option<bool> {
        match self {
            RtValue::Bool(v) => Some(v),
            _ => None,
        }
    }
    pub fn as_vec<Map, To>(self, map: Map) -> Option<Vec<To>>
    where
        Map: Fn(RtValue) -> To,
    {
        match self {
            RtValue::Array(elems) => Some(elems.into_iter().map(map).collect()),
            _ => None,
        }
    }
    pub fn as_map<Map, To>(self, map: Map) -> Option<HashMap<String, To>>
    where
        Map: Fn((String, RtValue)) -> (String, To),
    {
        match self {
            RtValue::Object(elems) => Some(HashMap::from_iter(
                elems.into_iter().map(map).collect::<Vec<_>>(),
            )),
            _ => None,
        }
    }
    pub fn as_int(self) -> Option<i64> {
        match self {
            RtValue::Number(RtValueNumber::Int(i)) => Some(i),
            _ => None,
        }
    }
    pub fn as_float(self) -> Option<f64> {
        match self {
            RtValue::Number(RtValueNumber::Float(f)) => Some(f),
            _ => None,
        }
    }

    pub fn as_pointer(self) -> Option<String> {
        match self {
            RtValue::Pointer(k) => Some(k),
            _ => None,
        }
    }

    /// tries to resolve the pointer to the value in BlackBoard,
    /// or if it is already a scalar value, then returns it
    pub fn with_ptr(self, ctx: TreeContextRef) -> RtResult<RtValue> {
        match self {
            RtValue::Pointer(p) => {
                ctx.bb()
                    .lock()?
                    .get(p.clone())?
                    .cloned()
                    .ok_or(RuntimeError::BlackBoardError(format!(
                        "The pointer {p} can not be processed (it is absent)"
                    )))
            }
            v => Ok(v),
        }
    }
}

impl From<Message> for RtValue {
    fn from(value: Message) -> Self {
        match value {
            Message::Num(n) => RtValue::Number(n.into()),
            Message::String(s) => RtValue::String(s.0),
            Message::Bool(b) => RtValue::Bool(b.into()),
            Message::Array(elems) => RtValue::Array(elems.into_iter().map(Into::into).collect()),
            Message::Object(elems) => {
                RtValue::Object(elems.into_iter().map(|(k, v)| (k, v.into())).collect())
            }
        }
    }
}

#[derive(Default, Debug, PartialEq, Clone, Deserialize, Serialize)]
pub struct RtArgs(pub Vec<RtArgument>);

impl RtArgs {
    /// takes the first one
    pub fn first(&self) -> Option<RtValue> {
        self.0.first().map(|a| a.value.clone())
    }
    /// takes the first one and transform
    pub fn first_as<M, To>(&self, map: M) -> Option<To>
    where
        M: Fn(RtValue) -> Option<To>,
    {
        self.0.first().and_then(|v| map(v.value.clone()))
    }

    /// finds by name
    pub fn find(&self, key: RtAKey) -> Option<RtValue> {
        self.0
            .iter()
            .find(|a| a.name == key)
            .map(|a| a.clone().value)
    }
    /// finds by name and transform
    pub fn find_as<M, To>(&self, key: RtAKey, map: M) -> Option<To>
    where
        M: Fn(RtValue) -> Option<To>,
    {
        self.find(key).and_then(|v| map(v.clone()))
    }

    /// finds by name or takes by index
    pub fn find_or_ith(&self, key: RtAKey, ith: usize) -> Option<RtValue> {
        self.0
            .iter()
            .find(|a| a.name == key)
            .or(self.0.get(ith))
            .map(|a| a.clone().value)
    }
    /// add to the given list of RtValues another one.
    /// # Notes
    /// If there is already a value with the same key, the value will be replaced
    pub fn with(self, key: &str, value: RtValue) -> RtArgs {
        let mut elems = self.0;
        let cursor = elems.iter().position(|e| e.clone().name() == key);
        let new_elem = RtArgument::new(key.to_string(), value);
        match cursor {
            None => {
                elems.push(new_elem);
            }
            Some(idx) => {
                elems.remove(idx);
                elems.insert(idx, new_elem)
            }
        }

        RtArgs(elems)
    }

    /// remove from the given list of RtValues another one.
    pub fn remove(self, key: &str) -> RtArgs {
        RtArgs(self.0.into_iter().filter(|v| v.name != key).collect())
    }
}

impl Display for RtArgs {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let elems = self.0.iter().map(|v| v.to_string()).join(",");
        f.write_str(elems.as_str())?;
        Ok(())
    }
}

/// The structure that represents the pair of the argument name and the value.
/// It is used in bb to store the arguments of the tree.
#[derive(Debug, PartialEq, Clone, Deserialize, Serialize)]
pub struct RtArgument {
    pub name: RtAKey,
    pub value: RtValue,
}

impl Display for RtValueNumber {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            RtValueNumber::Int(v) => f.write_str(format!("{}", v).as_str()),
            RtValueNumber::Float(v) => f.write_str(format!("{}", v).as_str()),
            RtValueNumber::Hex(v) => f.write_str(format!("0x{:02x}", v).as_str()),
            RtValueNumber::Binary(v) => f.write_str(format!("{:#b}", v).as_str()),
        }
    }
}

impl Display for RtValue {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            RtValue::String(v) => f.write_str(v)?,
            RtValue::Bool(b) => f.write_str(format!("{}", b).as_str())?,
            RtValue::Array(elems) => {
                let elems = elems.iter().map(|e| e.to_string()).join(",");
                f.write_str(format!("[{}]", elems).as_str())?;
            }
            RtValue::Object(obj) => {
                let elems = obj.iter().map(|(k, v)| format!("{}:{}", k, v)).join(",");
                f.write_str(format!("[{}]", elems).as_str())?;
            }
            RtValue::Number(n) => f.write_str(format!("{}", n).as_str())?,
            RtValue::Pointer(p) => f.write_str(format!("&{p}").as_str())?,
            RtValue::Call(_) => f.write_str("<Call>>")?,
        }
        Ok(())
    }
}

impl Display for RtArgument {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.write_str(format!("{}={}", &self.name, &self.value).as_str())?;
        Ok(())
    }
}

impl RtArgument {
    pub fn val(self) -> RtValue {
        self.value
    }
    pub fn name(self) -> RtAKey {
        self.name
    }
}

impl RtArgument {
    pub fn new(name: RtAKey, value: RtValue) -> Self {
        Self { name, value }
    }
    pub fn new_noname(value: RtValue) -> Self {
        Self {
            name: "arg".to_string(),
            value,
        }
    }

    /// tries to convert the argument to the given type
    /// # Notes
    /// If the argument is a pointer, then the value will be taken from parent arguments
    /// according to the parent parameters
    pub fn try_from(
        a: ArgumentRhs,
        p: Param,
        parent_args: Arguments,
        parent_params: Params,
    ) -> Result<(RtArgument, ArgumentRhs), TreeError> {
        RtArgument::validate_type(a.clone(), p.clone().tpe)?;
        match &a {
            ArgumentRhs::Id(id) => match find_arg_value(id, &parent_params, &parent_args).ok() {
                None => Ok((RtArgument::new(p.name, RtValue::Pointer(id.clone())), a)),
                Some(v) => RtArgument::try_from(v, p, Arguments::default(), Params::default()),
            },
            ArgumentRhs::Mes(m) => Ok((RtArgument::new(p.name, m.clone().into()), a)),
            ArgumentRhs::Call(c) => Ok((RtArgument::new(p.name, RtValue::Call(c.clone())), a)),
        }
    }
    /// validates the type of the argument in accordance with the type of the parameter
    pub fn validate_type(arg: ArgumentRhs, param: MesType) -> Result<(), TreeError> {
        let error = |lhs: &str, rhs: &str| {
            Err(cerr(format!(
                "the type of the given value '{lhs}' of the argument does not coincide to the type '{rhs}' of the definition ",
            )))
        };

        match (arg, param) {
            (ArgumentRhs::Call(_), MesType::Tree) => Ok(()),
            (ArgumentRhs::Call(_), m) => error("call", format!("{:?}", m).as_str()),
            (ArgumentRhs::Id(_), MesType::Tree) => error("pointer", "call"),
            (ArgumentRhs::Mes(_), MesType::Tree) => error("message", "call"),

            (ArgumentRhs::Id(_), _) => Ok(()),

            (ArgumentRhs::Mes(m), m_t) => {
                if m.same(&m_t) {
                    Ok(())
                } else {
                    error(format!("{}", m).as_str(), format!("{:?}", m_t).as_str())
                }
            }
        }
    }
}
