use crate::results::nodal_displacement::NodalDisplacement;
use crate::results::reaction::Reaction;
use crate::results::element_result::ElementResult;

pub struct TrussResult<'a>{
    pub displacements: Vec<NodalDisplacement>,
    pub reactions: Vec<Reaction<'a>>,
    pub element_results: Vec<ElementResult<'a>>
}