

use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtArgument, RtValue};
use crate::runtime::context::{TreeContextRef};
use crate::runtime::rtree::rnode::DecoratorType;
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::{RuntimeError, TickResult};
use crate::tests::{fb};

use crate::tree::project::Project;


#[test]
fn pointers() {
    let mut forester = fb("units/pointers").build().unwrap();
    let result = forester.run().unwrap();

    assert_eq!(result, TickResult::success())
}

#[test]
fn inter_args_func() {
    struct T;
    impl Impl for T {
        fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
            let v = args
                .first()
                .and_then(|v| v.as_string())
                .ok_or(RuntimeError::fail("expected a string".to_string()))?;

            ctx.bb().lock()?.put("t".to_string(), RtValue::str(v))?;

            Ok(TickResult::success())
        }
    }

    let mut fb = fb("units/inter_args");
    fb.register_sync_action("t", T);

    let mut forester = fb.build().unwrap();
    let result = forester.run().unwrap();

    assert_eq!(result, TickResult::success());

    let bb = forester.bb.lock().unwrap();
    let r = bb.get("t".to_string());
    assert_eq!(r, Ok(Some(&RtValue::str("test".to_string()))));
}

#[test]
fn inter_args_lambda() {
    let project = Project::build_from_text(
        r#"
root main test("test")
sequence test(a:string) sequence { t(a) } 
impl t(k:string);
        "#
        .to_string(),
    )
    .unwrap();

    let starter = RuntimeTree::build(project).unwrap();
    let tree = starter.tree;
    let node = tree.nodes.values().find(|n| n.is_name("t")).unwrap();

    assert_eq!(
        node.args().0,
        vec![RtArgument::new(
            "k".to_string(),
            RtValue::str("test".to_string())
        )]
    )
}

#[test]
fn inter_args() {
    let project = Project::build_from_text(
        r#"
root main test("test")
sequence test(a:string) t(a)
impl t(k:string);
        "#
        .to_string(),
    )
    .unwrap();

    let starter = RuntimeTree::build(project).unwrap();
    let tree = starter.tree;
    let node = tree.nodes.values().find(|n| n.is_name("t")).unwrap();

    assert_eq!(
        node.args().0,
        vec![RtArgument::new(
            "k".to_string(),
            RtValue::str("test".to_string())
        )]
    )
}

#[test]
fn inter_args_decorator() {
    let project = Project::build_from_text(
        r#"
root main test(100)
sequence test(a:num)  retry(a) t("a")
impl t(k:string);
        "#
        .to_string(),
    )
    .unwrap();

    let starter = RuntimeTree::build(project).unwrap();
    let tree = starter.tree;
    let node = tree
        .nodes
        .values()
        .find(|n| n.is_decorator(&DecoratorType::Retry))
        .unwrap();

    assert_eq!(
        node.args().0,
        vec![RtArgument::new_noname(RtValue::int(100))]
    )
}

#[test]
fn inter_args_ho() {
    let project = Project::build_from_text(
        r#"
root main test(100)
sequence test(a:num)  x(t(a))

sequence x(tr:tree) tr(..)
impl t(k:num);
        "#
        .to_string(),
    )
    .unwrap();

    let starter = RuntimeTree::build(project).unwrap();
    let tree = starter.tree;
    let node = tree.nodes.values().find(|n| n.is_name("t")).unwrap();
    assert_eq!(
        node.args().0,
        vec![RtArgument::new("k".to_string(), RtValue::int(100))]
    )
}

#[test]
fn inter_args_pointers() {
    let project = Project::build_from_text(
        r#"
import "std::actions"
root main {
    store("t","test")
    test(t)
}
sequence test(a:string) t(a)
impl t(k:string);
        "#
        .to_string(),
    )
    .unwrap();

    let starter = RuntimeTree::build(project).unwrap();
    let tree = starter.tree;
    let node = tree.nodes.values().find(|n| n.is_name("t")).unwrap();

    assert_eq!(
        node.args().0,
        vec![RtArgument::new(
            "k".to_string(),
            RtValue::Pointer("t".to_string())
        )]
    )
}
