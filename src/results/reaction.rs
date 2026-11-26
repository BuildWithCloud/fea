use crate::model::force::Force;

#[derive(Debug)]
pub struct Reaction<const D: usize>{
    pub node_id: usize,
    pub force: Force<D>,
}
