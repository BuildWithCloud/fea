use crate::results::nodal_displacement::NodalDisplacement;
use crate::results::reaction::Reaction;
use crate::results::element_result::ElementResult;

pub struct TrussResult<const D: usize>{
    pub displacements: Vec<NodalDisplacement<D>>,
    pub reactions: Vec<Reaction<D>>,
    pub element_results: Vec<ElementResult>
}
