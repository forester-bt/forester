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
    use crate::tree::parser::Parser;

    #[test]
    fn script() {
        let script = r#"
// built in library        
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
        assert_eq!(result.0.len(), 12);
    }
}
