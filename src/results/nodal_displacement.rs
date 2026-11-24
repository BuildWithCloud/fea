use crate::Node;

#[derive(Debug)]
pub struct NodalDisplacement<'a> {
    pub node: &'a Node,
    pub x_displacement: f64,
    pub y_displacement: f64,
}