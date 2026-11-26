use crate::results::displacement::Displacement;

#[derive(Debug)]
pub struct NodalDisplacement<const D: usize> {
    pub node_id: usize,
    pub displacement: Displacement<D>,
}
