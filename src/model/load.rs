use crate::model::force::Force;

pub struct Load<const D: usize>{
    pub node_id: usize,
    pub force: Force<D>,
}
