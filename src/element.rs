use nalgebra::Matrix4;

use crate::node::Node;

pub struct Element<'a>{
    pub node_1: &'a Node,
    pub node_2: &'a Node,
    pub youngs_modulus: f64,
    pub area: f64,
}

impl Element<'_>{
    pub fn theta(& self) -> f64{
        let delta_y = self.node_2.y_position - self.node_1.y_position;
        let delta_x = self.node_2.x_position - self.node_1.x_position;
        delta_y.atan2(delta_x)
    }

    pub fn length(& self) -> f64{
        let delta_y = self.node_2.y_position - self.node_1.y_position;
        let delta_x = self.node_2.x_position - self.node_1.x_position;
        (delta_x.powi(2) + delta_y.powi(2)).sqrt()
    }

    pub fn stiffness_matrix(& self) -> Matrix4<f64>{
        let l = self.length();
        let theta = self.theta();
        let c = theta.cos();
        let s = theta.sin();
        let k = self.youngs_modulus * self.area / l;
        Matrix4::from_row_slice(&[
            k*c*c, k*c*s, -k*c*c, -k*c*s,
            k*c*s, k*s*s, -k*c*s, -k*s*s,
            -k*c*c, -k*c*s, k*c*c, k*c*s,
            -k*c*s, -k*s*s, k*c*s, k*s*s,
        ])
    }
}
