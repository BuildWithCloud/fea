use crate::node::Node;

pub struct Reaction<'a>{
    pub node: &'a Node,
    pub x_force: f64,
    pub y_force: f64,
}