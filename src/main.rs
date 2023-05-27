fn main() {
    println!("Hello, world!");
}


enum Status {
    Running,
    Failure,
    Success
}

struct ActionError;

struct NodeCtx;

impl NodeCtx {
    fn read<T:Deserialize>(&self, key:String) -> Result<T,ActionError>{
        Ok("")
    }
}

trait Action {

    fn tick(&mut self, ctx:&mut NodeCtx) -> Result<Status,ActionError>;

}
