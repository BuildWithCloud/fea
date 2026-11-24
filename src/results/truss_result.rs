use crate::results::nodal_displacement::NodalDisplacement;
use crate::results::reaction::Reaction;
use crate::results::element_result::ElementResult;

pub struct TrussResult{
    pub displacements: Vec<NodalDisplacement>,
    pub reactions: Vec<Reaction>,
    pub element_results: Vec<ElementResult>
}
