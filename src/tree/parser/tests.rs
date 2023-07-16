#[cfg(test)]
mod arguments;
#[cfg(test)]
mod call;
#[cfg(test)]
mod definition;
#[cfg(test)]
mod file;
#[cfg(test)]
mod import;
#[cfg(test)]
mod message;
#[cfg(test)]
mod params;

#[cfg(test)]
mod tests {
    use crate::tree::parser::ast::arg::{Argument, Arguments};
    use crate::tree::parser::ast::call::Call;
    use crate::tree::parser::ast::message::{Bool, Message, Number, StringLit};
    use crate::tree::parser::ast::Key;
    use crate::tree::parser::Parser;
    use parsit::test::parser_test::*;
    use std::fs;
    use std::path::PathBuf;

    pub fn load_file(path: &str) -> String {
        let mut ex = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        for next in path.split("/") {
            match next {
                ".." => ex = ex.parent().unwrap().into(),
                n => ex.push(n),
            }
        }
        fs::read_to_string(ex).unwrap()
    }

    #[test]
    fn smoke() {
        let script = load_file("tree/tests/plain_project/main.tree");
        let parser = Parser::new(script.as_str()).unwrap();
        let result = parser.parse().unwrap();
    }
}
