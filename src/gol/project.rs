use std::borrow::Cow;
use std::collections::HashMap;
use std::fs;
use std::iter::Map;
use std::path::PathBuf;
use parsit::error::ParseError;
use crate::gol::ast::{AstFile, Import, Tree};
use crate::gol::GolError;
use crate::gol::parser::Parser;




#[derive(Debug, Default, Clone)]
pub struct Project {
    files: HashMap<String, File>,
}


pub fn file_to_string(file_path: PathBuf) -> std::io::Result<String> {
    fs::read_to_string(file_path)
}


impl<'a> Project {
    pub fn build(main_file: String) -> Result<Project, GolError> {


        let files = parse_file_with_imports(main_file,HashMap::new())?;


        Ok(Project{files})
    }
}

pub fn parse_file_with_imports(file:String, mut files:HashMap<String,File>) -> Result<HashMap<String,File>,GolError>{
    let text =
        file_to_string(PathBuf::from(file))
            .map_err(|e| ParseError::ExternalError(e.to_string(), 0))?;
    let file = Parser::new(text.as_str())?.parse()?;
    Ok(files)
}



#[derive(Debug, Default, Clone)]
pub struct File {
    pub name: String,
    pub imports: HashMap<String, Import>,
    pub definitions: HashMap<String, Tree>,
}

