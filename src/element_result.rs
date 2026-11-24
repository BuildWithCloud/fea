use crate::node::Node;

pub struct ElementResult<'a>{
    pub node_1: &'a Node,
    pub node_2: &'a Node,
    pub axial_force: f64,
    pub stress: f64,
    pub strain: f64,
}

impl ElementResult<'_> {
    pub fn theta(& self) -> f64{
        let delta_y = self.node_2.y_position - self.node_1.y_position;
        let delta_x = self.node_2.x_position - self.node_1.x_position;
        delta_y.atan2(delta_x)
    }   
}