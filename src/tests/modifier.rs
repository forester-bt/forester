use crate::runtime::builder::ForesterBuilder;
use crate::tests::test_folder;

#[test]
fn smoke() {
    let root = test_folder("modify/smoke");

    let mut fb = ForesterBuilder::from_fs();
    fb.main_file("main.tree".to_string());
    fb.root(root);

    let forester = fb.build().unwrap();
    let mq = forester.mod_queue.clone();
}
