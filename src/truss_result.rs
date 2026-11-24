use crate::nodal_displacement::NodalDisplacement;
use crate::reaction::Reaction;
use crate::element_result::ElementResult;

pub struct TrussResult<'a>{
    pub displacements: Vec<NodalDisplacement>,
    pub reactions: Vec<Reaction<'a>>,
    pub element_results: Vec<ElementResult<'a>>
}