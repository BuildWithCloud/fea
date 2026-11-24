use nalgebra::{Matrix4, Vector4};

use crate::model::node::Node;

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

    pub fn axial_stiffness(&self) -> f64{
        self.youngs_modulus * self.area / self.length()
    }

    pub fn transformation_matrix(& self) -> Matrix4<f64>{
        let theta = self.theta();
        let c = theta.cos();
        let s = theta.sin();
        Matrix4::from_row_slice(&[
            c, -s, 0.0, 0.0,
            s, c, 0.0, 0.0,
            0.0, 0.0, c, -s,
            0.0, 0.0, s, c,
        ])
    }

    pub fn local_stiffness_matrix(&self) -> Matrix4<f64>{
        let k = self.axial_stiffness();
        Matrix4::from_row_slice(&[
            k, 0.0, -k, 0.0,
            0.0, 0.0, 0.0, 0.0,
            -k, 0.0, k, 0.0,
            0.0, 0.0, 0.0, 0.0,
        ])
    }

    pub fn local_displacements(&self, global_displacements: &Vector4<f64>) -> Vector4<f64>{
        self.transformation_matrix().transpose() * global_displacements
    }

    pub fn local_forces(&self, global_displacements: &Vector4<f64>) -> Vector4<f64>{
        self.local_stiffness_matrix() * self.local_displacements(global_displacements)
    }

    pub fn axial_force(&self, global_displacements: &Vector4<f64>) -> f64{
        self.local_forces(global_displacements)[2]
    }

    pub fn stress(&self, global_displacements: &Vector4<f64>) -> f64{
        self.axial_force(global_displacements) / self.area
    }

    pub fn strain(&self, global_displacements: &Vector4<f64>) -> f64{
        let local_displacements = self.local_displacements(global_displacements);
        let new_length = local_displacements[0] -local_displacements[2];
        let old_length = self.length();
        return new_length/old_length;
    }
}
