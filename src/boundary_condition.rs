use crate::node::Node;

pub struct BoundaryCondition<'a>{
    pub node: &'a Node,
    pub x_fixed: bool,
    pub y_fixed: bool
}
