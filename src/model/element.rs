use nalgebra::{Matrix4, Vector4};

use crate::model::node::Node;

pub struct Element{
    pub node_1_id: usize,
    pub node_2_id: usize,
    pub youngs_modulus: f64,
    pub area: f64,
}

/// An element together with the two nodes it connects, so geometry-dependent
/// calculations have the positions they need. Create one with `Truss::element_views`.
pub struct ElementView<'a, const D: usize>{
    pub element: &'a Element,
    pub node_1: &'a Node<D>,
    pub node_2: &'a Node<D>,
}

impl ElementView<'_, 2>{
    pub fn theta(& self) -> f64{
        let delta_y = self.node_2.position.y - self.node_1.position.y;
        let delta_x = self.node_2.position.x - self.node_1.position.x;
        delta_y.atan2(delta_x)
    }

    pub fn length(& self) -> f64{
        let delta_y = self.node_2.position.y - self.node_1.position.y;
        let delta_x = self.node_2.position.x - self.node_1.position.x;
        (delta_x.powi(2) + delta_y.powi(2)).sqrt()
    }

    pub fn stiffness_matrix(& self) -> Matrix4<f64>{
        let l = self.length();
        let theta = self.theta();
        let c = theta.cos();
        let s = theta.sin();
        let k = self.element.youngs_modulus * self.element.area / l;
        Matrix4::from_row_slice(&[
            k*c*c, k*c*s, -k*c*c, -k*c*s,
            k*c*s, k*s*s, -k*c*s, -k*s*s,
            -k*c*c, -k*c*s, k*c*c, k*c*s,
            -k*c*s, -k*s*s, k*c*s, k*s*s,
        ])
    }

    pub fn axial_stiffness(&self) -> f64{
        self.element.youngs_modulus * self.element.area / self.length()
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
        self.axial_force(global_displacements) / self.element.area
    }

    pub fn strain(&self, global_displacements: &Vector4<f64>) -> f64{
        let local_displacements = self.local_displacements(global_displacements);
        let new_length = local_displacements[0] -local_displacements[2];
        let old_length = self.length();
        return new_length/old_length;
    }
}
