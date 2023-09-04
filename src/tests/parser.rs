use crate::tests::test_folder;
use crate::tree::parser::Parser;
use std::fs;

#[test]
fn smoke() {
    let path = test_folder("plain_project/main.tree");
    let scr = fs::read_to_string(path).unwrap();
    let parser = Parser::new(scr.as_str()).unwrap();
    let result = parser.parse().unwrap();
    assert_eq!(result.0.len(), 6);
}
