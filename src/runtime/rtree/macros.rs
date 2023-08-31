/// Creates a list of arguments.
///
/// #Example
///
/// ```
///
/// use std::collections::HashMap;
/// use forester_rs::*;
/// use forester_rs::runtime::args::{RtArgs, RtArgument, RtValue};
/// use forester_rs::runtime::args::RtValueNumber;
/// #[test]
/// fn test(){
///      assert_eq!(
///             args!(arg!("a" , rt_str!("a")), arg!("b" , rt_num!(i 1))),
///             RtArgs(vec![
///                 RtArgument::new("a".to_string(), RtValue::str("a".to_string())),
///                 RtArgument::new("b".to_string(), RtValue::int(1)),
///             ])
///         )
/// }
/// ```
#[macro_export]
macro_rules! args {
    [] => {{
       RtArgs(vec![])
    }};
    () => {{
       RtArgs(vec![])
    }};
    [$($rt_elems:expr),+  ] => {{
        let mut elems = Vec::new();
        $( elems.push($rt_elems) ; )+
        RtArgs(elems)
    }};
    ($($rt_elems:expr),+ ) => {{
        let mut elems = Vec::new();
        $( elems.push($rt_elems) ; )+
        RtArgs(elems)
    }};

}

/// Creates an argument RtArgument.
/// Accepts a string and a value
///
/// #Example
///
/// ```
///
/// use std::collections::HashMap;
/// use forester_rs::*;
/// use forester_rs::runtime::args::{RtArgument, RtValue};
/// use forester_rs::runtime::args::RtValueNumber;
/// #[test]
/// fn test(){
///      let x = arg!("a" , rt_array![rt_num!(i 1), rt_str!("a")]);
///         assert_eq!(
///             x,
///             RtArgument::new(
///                 "a".to_string(),
///                 RtValue::Array(vec![RtValue::int(1), RtValue::str("a".to_string())])
///             )
///         );
/// }
/// ```
/// ```
/// use std::collections::HashMap;
/// use forester_rs::*;
/// use forester_rs::runtime::args::{RtArgument, RtValue};
/// use forester_rs::runtime::args::RtValueNumber;
///
///  fn test(){
///         assert_eq!(
///             arg!("a" , rt_obj!{
///                 "l" => rt_array![rt_str!("a"),rt_str!("b")]
///                 "r" => rt_num!(i 15)
///             }),
///             RtArgument::new(
///                 "a".to_string(),
///                 RtValue::Object(HashMap::from_iter(vec![
///                     (
///                         "l".to_string(),
///                         RtValue::Array(vec![
///                             RtValue::str("a".to_string()),
///                             RtValue::str("b".to_string())
///                         ])
///                     ),
///                     ("r".to_string(), RtValue::int(15)),
///                 ]))
///             )
///         );
/// }
///
/// ```
#[macro_export]
macro_rules! arg {
    ($name:expr , $value:expr) => {{
        RtArgument::new($name.to_string(), $value)
    }};
}

/// RtValue::String that accepts &str as argument
#[macro_export]
macro_rules! rt_str {
    ($value:expr) => {{
        RtValue::str($value.to_string())
    }};
}

/// RtValue::Pointer that accepts &str as argument
#[macro_export]
macro_rules! rt_ptr {
    ($value:expr) => {{
        RtValue::Pointer($value.to_string())
    }};
}

/// RtValue::Bool that accepts true | false  as argument
#[macro_export]
macro_rules! rt_bool {
    (true) => {{
        RtValue::Bool(true)
    }};
    (false) => {{
        RtValue::Bool(false)
    }};
}

/// RtValue::Number that accepts (i i64) | (f f64) | (b usize) (h usize)  as argument
#[macro_export]
macro_rules! rt_num {
    (i$value:expr) => {{
        RtValue::Number(RtValueNumber::Int($value))
    }};
    (f$value:expr) => {{
        RtValue::Number(RtValueNumber::Float($value))
    }};
    (b$value:expr) => {{
        RtValue::Number(RtValueNumber::Binary($value))
    }};
    (h$value:expr) => {{
        RtValue::Number(RtValueNumber::Hex($value))
    }};
}

/// RtValue::Array that has the same behaviour as an array
#[macro_export]
macro_rules! rt_array {
    [] => {{
        RtValue::Array(Vec::new())
    }};
    () => {{
        RtValue::Array(Vec::new())
    }};
    [$($rt_elems:expr),+  ] => {{
        let mut elems = Vec::new();
        $( elems.push($rt_elems) ; )+
        RtValue::Array(elems)
    }};
    ($($rt_elems:expr),+ ) => {{
        let mut elems = Vec::new();
        $( elems.push($rt_elems) ; )+
        RtValue::Array(elems)
    }};

}

/// RtValue::Object that has the same behaviour as an json_object
#[macro_export]
macro_rules! rt_obj {
    {} => {{
        RtValue::Object(HashMap::new())
    }};
    () => {{
       RtValue::Object(HashMap::new())
    }};

    {$($name:expr => $val:expr)+ } => {{
        let mut elems = Vec::new();
        $( elems.push(($name.to_string(),$val)) ; )+
        RtValue::Object(HashMap::from_iter(elems))
    }}
}

/// Creates RNodeName
///
/// # Example
/// ```
/// use std::collections::HashMap;
/// use forester_rs::*;
/// use forester_rs::runtime::args::{RtArgument, RtValue};
/// use forester_rs::runtime::args::RtValueNumber;
/// use forester_rs::runtime::rtree::rnode::{FlowType, RNodeName};
///
///  fn test(){
///         assert_eq!(
///           node_name!(),
///           RNodeName::Lambda  
///         );
///         assert_eq!(
///           node_name!("name"),
///           RNodeName::Name("name".to_string())  
///         );
///         assert_eq!(
///           node_name!("name", "alias"),
///           RNodeName::Alias("name".to_string(),"alias".to_string())  
///         );
///
/// }
///
/// ```
#[macro_export]
macro_rules! node_name {
    () => {{
        RNodeName::Lambda
    }};
    ($name:expr) => {{
        RNodeName::Name($name.to_string())
    }};
    ($name:expr, $alias:expr) => {{
        RNodeName::Alias($name.to_string(), $alias.to_string())
    }};
}

/// Creates RtNodeBuilder::Leaf(..)
///
/// # Example
/// ```
/// use forester_rs::runtime::args::RtArgs;
/// use std::collections::HashMap;
/// use forester_rs::*;
/// use forester_rs::runtime::args::{RtArgument, RtValue};
/// use forester_rs::runtime::args::RtValueNumber;
/// use forester_rs::runtime::rtree::builder::RtNodeBuilder;
/// use forester_rs::runtime::rtree::rnode::RNodeName;
///
///  fn test(){
///         let action = action!();
///         let action = action!(node_name!("name"));
///         let action = action!(node_name!("name"), args!());
///         
/// }
///
/// ```
#[macro_export]
macro_rules! action {
    () => {{
        RtNodeBuilder::leaf(node_name!(), args!())
    }};
    ($name:expr) => {{
        RtNodeBuilder::leaf($name, args!())
    }};

    ($name:expr,$args:expr) => {{
        RtNodeBuilder::leaf($name, $args)
    }};
}

/// Creates RtNodeBuilder::flow(..)
/// The syntax is the following:
///  with enlisting
/// `flow!(type name, args; children...)`
///  
///  with given vec
///  `flow!(type name, args, children_vec)`
/// # Example
/// ```
/// use forester_rs::runtime::args::RtArgs;
/// use std::collections::HashMap;
/// use forester_rs::*;
/// use forester_rs::runtime::args::{RtArgument, RtValue};
/// use forester_rs::runtime::args::RtValueNumber;
/// use forester_rs::runtime::rtree::builder::RtNodeBuilder;
/// use forester_rs::runtime::rtree::rnode::{FlowType, RNodeName};
///
///  fn test(){
///         
///         let flow_with_enlist = flow!(fallback node_name!(), args!();
///                         action!(),
///                         action!(),
///                         action!(),
///                         action!()
///                     );
///
///         let actions = vec![ action!(), action!(), action!() ];
///         let flow_with_vec = flow!(fallback node_name!(), args!(), actions);
///
/// }
///
/// ```
#[macro_export]
macro_rules! flow {
    (root $name:expr, $args:expr; $($children:expr),+ ) => {{
        let mut elems = Vec::new();
        $( elems.push($children.into()) ; )+

        RtNodeBuilder::flow(FlowType::Root, $name, $args, elems)
    }};
    (root $name:expr, $args:expr, $children:expr ) => {{
        let elems = $children.into_iter().map(|v|v.into()).collect();
        RtNodeBuilder::flow(FlowType::Root, $name, $args, elems)
    }};
    (parallel $name:expr, $args:expr; $($children:expr),+ ) => {{
        let mut elems = Vec::new();
        $( elems.push($children.into()) ; )+

        RtNodeBuilder::flow(FlowType::Parallel, $name, $args, elems)
    }};
    (parallel $name:expr, $args:expr, $children:expr) => {{
       let elems = $children.into_iter().map(|v|v.into()).collect();
        $( elems.push($children.into()) ; )+

        RtNodeBuilder::flow(FlowType::Parallel, $name, $args, elems)
    }};

    (sequence $name:expr, $args:expr; $($children:expr),+ ) => {{
        let mut elems = Vec::new();
        $( elems.push($children.into()) ; )+

        RtNodeBuilder::flow(FlowType::Sequence, $name, $args, elems)
    }};
    (sequence $name:expr, $args:expr, $children:expr) => {{
        let elems = $children.into_iter().map(|v|v.into()).collect();

        RtNodeBuilder::flow(FlowType::Sequence, $name, $args, elems)
    }};
    (m_sequence $name:expr, $args:expr; $($children:expr),+ ) => {{
        let mut elems = Vec::new();
        $( elems.push($children.into()) ; )+

        RtNodeBuilder::flow(FlowType::MSequence, $name, $args, elems)
    }};
    (m_sequence $name:expr, $args:expr, $children:expr) => {{
        let elems = $children.into_iter().map(|v|v.into()).collect();

        RtNodeBuilder::flow(FlowType::MSequence, $name, $args, elems)
    }};
    (r_sequence $name:expr, $args:expr; $($children:expr),+ ) => {{
        let mut elems = Vec::new();
        $( elems.push($children.into()) ; )+

        RtNodeBuilder::flow(FlowType::RSequence, $name, $args, elems)
    }};
    (r_sequence $name:expr, $args:expr, $children:expr) => {{
        let elems = $children.into_iter().map(|v|v.into()).collect();

        RtNodeBuilder::flow(FlowType::RSequence, $name, $args, elems)
    }};
    (fallback $name:expr, $args:expr; $($children:expr),+ ) => {{
        let mut elems = Vec::new();
        $( elems.push($children.into()) ; )+

        RtNodeBuilder::flow(FlowType::Fallback, $name, $args, elems)
    }};
    (fallback $name:expr, $args:expr, $children:expr) => {{
        let elems = $children.into_iter().map(|v|v.into()).collect();

        RtNodeBuilder::flow(FlowType::Fallback, $name, $args, elems)
    }};
    (r_fallback $name:expr, $args:expr; $($children:expr),+ ) => {{
        let mut elems = Vec::new();
        $( elems.push($children.into()) ; )+

        RtNodeBuilder::flow(FlowType::RFallback, $name, $args, elems)
    }};
    (r_fallback $name:expr, $args:expr, $children:expr) => {{
        let elems = $children.into_iter().map(|v|v.into()).collect();

        RtNodeBuilder::flow(FlowType::RFallback, $name, $args, elems)
    }};
}

/// Creates RtNodeBuilder::decorator(..)
///
/// # Example
/// ```
/// use forester_rs::runtime::args::RtArgs;
/// use std::collections::HashMap;
/// use forester_rs::*;
/// use forester_rs::runtime::args::{RtArgument, RtValue};
/// use forester_rs::runtime::args::RtValueNumber;
/// use forester_rs::runtime::rtree::builder::RtNodeBuilder;
/// use forester_rs::runtime::rtree::rnode::{FlowType,DecoratorType, RNodeName};
///
///  fn test(){
///         
///         let decorator = decorator!(
///                         inverter args!(),
///                         flow!(fallback node_name!(), args!(); action!())
///                      );
/// }
///
/// ```
#[macro_export]
macro_rules! decorator {
    (inverter $args:expr, $child:expr ) => {{
        RtNodeBuilder::decorator(DecoratorType::Inverter, $args, $child.into())
    }};
    (force_success $args:expr, $child:expr ) => {{
        RtNodeBuilder::decorator(DecoratorType::ForceSuccess, $args, $child.into())
    }};
    (force_fail $args:expr, $child:expr ) => {{
        RtNodeBuilder::decorator(DecoratorType::ForceFail, $args, $child.into())
    }};
    (repeat $args:expr, $child:expr ) => {{
        RtNodeBuilder::decorator(DecoratorType::Repeat, $args, $child.into())
    }};
    (retry $args:expr, $child:expr ) => {{
        RtNodeBuilder::decorator(DecoratorType::Retry, $args, $child.into())
    }};
    (timeout $args:expr, $child:expr ) => {{
        RtNodeBuilder::decorator(DecoratorType::Timeout, $args, $child.into())
    }};
    (delay $args:expr, $child:expr ) => {{
        RtNodeBuilder::decorator(DecoratorType::Delay, $args, $child.into())
    }};
}

#[cfg(test)]
mod tests {
    use crate::runtime::args::{RtArgs, RtArgument, RtValue, RtValueNumber};
    use std::collections::HashMap;

    #[test]
    fn arg() {
        let x = arg!("a", rt_str!("v"));
        assert_eq!(
            x,
            RtArgument::new("a".to_string(), RtValue::str("v".to_string()))
        );

        let x = arg!("a", rt_bool!(true));
        assert_eq!(x, RtArgument::new("a".to_string(), RtValue::Bool(true)));

        let x = arg!("a", rt_num!(i 10));
        assert_eq!(x, RtArgument::new("a".to_string(), RtValue::int(10)));

        let x = arg!("a", rt_array![]);
        assert_eq!(x, RtArgument::new("a".to_string(), RtValue::Array(vec![])));

        assert_eq!(
            arg!(
                "a",
                rt_obj! {
                "l" => rt_array![rt_str!("a"),rt_str!("b")]
                "r" => rt_num!(i 15)
                }
            ),
            RtArgument::new(
                "a".to_string(),
                RtValue::Object(HashMap::from_iter(vec![
                    (
                        "l".to_string(),
                        RtValue::Array(vec![
                            RtValue::str("a".to_string()),
                            RtValue::str("b".to_string())
                        ])
                    ),
                    ("r".to_string(), RtValue::int(15)),
                ]))
            )
        );

        assert_eq!(
            args!(arg!("a", rt_str!("a")), arg!("b", rt_num!(i 1))),
            RtArgs(vec![
                RtArgument::new("a".to_string(), RtValue::str("a".to_string())),
                RtArgument::new("b".to_string(), RtValue::int(1)),
            ])
        )
    }
}
