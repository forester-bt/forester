use crate::runtime::args::RtValue;
use crate::runtime::blackboard::{BBRef, BBKey};
use crate::runtime::RtOk;


/// Pushes the value to the array in the cell.
/// If the cell is absent it will be created.
/// If the cell is not an array it will be converted
/// to the array with the current value and the new value.
pub fn push_to_arr(bb: BBRef, key: BBKey, value: RtValue) -> RtOk {
    let mut bb = bb.lock()?;

    let value = match bb.get(key.clone())? {
        None => {
            RtValue::Array(vec![value])
        }
        Some(RtValue::Array(elems)) => {
            let mut elems = elems.clone();
            elems.push(value);
            RtValue::Array(elems)
        }
        Some(v) => {
            RtValue::Array(vec![v.clone(), value])
        }
    };


    bb.put(key, value)
}