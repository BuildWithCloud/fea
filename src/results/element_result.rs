use crate::model::node::Node;

pub struct ElementResult<'a>{
    pub node_1: &'a Node,
    pub node_2: &'a Node,
    pub axial_force: f64,
    pub stress: f64,
    pub strain: f64,
}

impl ElementResult<'_> {
    pub fn theta(& self) -> f64{
        let delta_y = self.node_2.position.y - self.node_1.position.y;
        let delta_x = self.node_2.position.x - self.node_1.position.x;
        delta_y.atan2(delta_x)
    }   
}