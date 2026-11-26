use crate::model::position::Position;

#[derive(Debug)]
pub struct Node<const D: usize>{
    pub id: usize,
    pub position: Position<D>,
}
