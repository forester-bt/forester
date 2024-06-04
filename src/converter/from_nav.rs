use std::path::PathBuf;
use quick_xml::events::{BytesStart, Event};
use quick_xml::reader::Reader;
use crate::converter::Converter;
use crate::read_file;


use crate::runtime::{RtResult, RuntimeError};
use crate::runtime::args::{RtArgument, RtValue};
use crate::runtime::builder::ros_nav::{find_ros_action, RosAction, RosParam};

use crate::tree::parser::ast::arg::MesType;


#[cfg(windows)]
const LINE_ENDING: &'static str = "\r\n";
#[cfg(not(windows))]
const LINE_ENDING: &'static str = "\n";

/// The struct is used to convert the xml file from the nav2 format into the runtime tree.
/// #Notes
/// For now, there is no any validations are performed.
/// The converter assumes that the xml file is correct.
///
/// Importantly it assumes all arguments to be string literals.
pub struct FromNav2 {
    xml: String,
}

impl FromNav2 {
    pub fn new(xml: String) -> Self {
        Self { xml }
    }
    pub fn read_file(xml: &PathBuf) -> RtResult<Self> {
        Ok(read_file(xml).map(|r| Self::new(r))?)
    }


    pub fn reader(&self) -> Reader<&[u8]> {
        let mut reader = Reader::from_str(&self.xml);
        reader.trim_text(true);
        reader
    }
}

impl Converter for FromNav2 {
    type Output = String;

    fn convert(&self) -> RtResult<Self::Output> {
        let mut reader = self.reader();
        let mut res = String::new();
        // let mut stack = Vec::new();
        let indent = 0;
        loop {
            match reader.read_event() {
                Ok(Event::Eof) => break,
                Ok(Event::Empty(e)) => {
                    let line = handle_terminal(e)?;
                    let line = format!("{line:>indent$}{LINE_ENDING}");
                    res.push_str(line.as_str());
                }
                Err(e) => return Err(RuntimeError::ExportError(format!("Error at position {}: {:?}", reader.buffer_position(), e))),
                _ => (),
            }
        }

        Ok(res)
    }
}

fn handle_terminal(e: BytesStart) -> RtResult<String> {
    let name = String::from_utf8(e.name().0.into())?;
    let action = find_ros_action(name.as_str())
        .ok_or(RuntimeError::WrongArgument(format!(r#"ros analogue not found for node {:?}. Check the import "ros::nav2""#, name)))?;
    let mut args = vec![];

    for attr_res in e.attributes() {
        let attr = attr_res?;
        let key = String::from_utf8(attr.key.0.to_vec())?;
        let v_str = String::from_utf8(attr.value.to_vec())?;
        if v_str.starts_with("{") && v_str.ends_with("}") {
            let value = v_str.trim_start_matches("{").trim_end_matches("}");
            args.push(RtArgument::new(key, RtValue::Pointer(value.to_string())).to_string());
        } else {
            let param = find_action(&action, key.clone())?;
            let argument = RtArgument::new(key, convert_arg(&action, v_str, param)?);
            args.push(argument.to_string());
        }
    }

    Ok(format!("{}({})", action.name, args.join(", ")))
}

fn convert_arg(action: &RosAction, v_str: String, param: &RosParam) -> Result<RtValue, RuntimeError> {
    match param.tpe() {
        MesType::Num => Ok(RtValue::float(parse_float(&action, v_str)?)),
        MesType::String => Ok(RtValue::str(v_str)),
        MesType::Bool => Ok(RtValue::Bool(v_str.parse::<bool>()?)),
        e =>
            Err(RuntimeError::WrongArgument(format!("unexpected argument type {} for action {}", e, action.name))),
    }
}

fn parse_float(action: &RosAction, v_str: String) -> Result<f64, RuntimeError> {
    v_str.parse::<f64>()
        .map_err(|e|
            RuntimeError::ExportError(format!("Error parsing float value {} for action {}. Error: {}", v_str, action.name, e)))
}

fn find_action(action: &RosAction, key: String) -> Result<&RosParam, RuntimeError> {
    action.params
        .iter()
        .find(|p| p.test(key.as_str()))
        .ok_or(RuntimeError::WrongArgument(format!("unexpected argument {} for action {}", key, action.name)))
}