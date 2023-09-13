use crate::runtime::rtree::RuntimeTree;
use crate::runtime::RtResult;

/// The module is responsible to transform the runtime tree into the specific given format and the vice versa.


trait Exporter {
    type Into;
    /// Exports the given tree into the specific format.
    fn export(&self, tree: &RuntimeTree) -> RtResult<Self::Into>;
}
