use crate::model::node::Node;

pub struct Load<'a>{
    pub node: &'a Node,
    pub x_force: f64,
    pub y_force: f64
}
