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

    #[test]
    fn script() {
        let script = r#"
        import "std:actions"

r_fallback retry_with_delay(delay:num, attempts:num,trg:tree){
    retry(attempts) fallback {
        trg(..)
        wait(delay)
        fail()
    }
    wait(delay)
}

r_sequence if_else(test:tree, then:tree, else:tree){
    r_fallback{
        test(..)
        else(..)
    }
    then(..)
}


r_fallback with_flag(action:tree){
    stop_flag()
    action(..)
}


cond is_hazard_nearby();
cond on_target(target:object);

impl do_job();
impl wait(ms:num);
impl move_to(target:object);
impl charge();

r_fallback to_charger(charger_station:object){
    retry_with_delay(1000, 10,
        r_sequence {
            if_else(
                is_hazard_nearby(),
                wait(1000),
                retry_with_delay(1000, 10, move_to(charger_station))
            )
            if_else(
                on_target(charger_station),
                charge(),
                retry_with_delay(1000, 10, move_to(charger_station))
            )
        }
    )
    running()
}


root main r_fallback {
    if_else(
        test = battery_low(),
        then = with_flag(to_charger({"x":10,"y":10})),
        else = do_job()
    )
}        
        "#;

        let parser = Parser::new(script).unwrap();
        let result = parser.parse().unwrap();
    }
}
