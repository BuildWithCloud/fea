use nalgebra::{DMatrix, DVector};

use crate::boundary_condition::BoundaryCondition;
use crate::element::Element;
use crate::load::Load;
use crate::node::Node;

pub struct Truss<'a>{
    pub nodes: Vec<&'a Node>,
    pub elements: Vec<&'a Element<'a>>,
    pub loads: Vec<&'a Load<'a>>,
    pub boundary_conditions: Vec<&'a BoundaryCondition<'a>>,
}
impl Truss<'_>{
    pub fn calculate_global_stiffness_matrix(&self) -> DMatrix<f64>{
        let number_of_dof = self.nodes.len() * 2;
        let mut k_global = DMatrix::zeros(number_of_dof, number_of_dof);

        for element in self.elements.iter(){
            let global_elemental_stiffness_matrix = element.stiffness_matrix();

            let node_1_index = self.node_index(element.node_1.id);
            let node_2_index = self.node_index(element.node_2.id);
            let dof = [node_1_index * 2, node_1_index * 2 + 1, node_2_index * 2, node_2_index * 2 + 1];

            for i in 0..4{
                for j in 0..4{
                    k_global[(dof[i], dof[j])] += global_elemental_stiffness_matrix[(i, j)];
                }
            }
        }
        k_global
    }

    fn calculate_cut_global_stiffness_matrix(&self, global_stiffness_matrix: DMatrix<f64>) -> DMatrix<f64>{
        let remove_dof = self.calculate_dofs_to_cut();
        let retained_dofs: Vec<usize> = remove_dof.iter()
            .enumerate()
            .filter_map(|(index, removed)| (!removed).then_some(index))
            .collect();

        DMatrix::from_fn(retained_dofs.len(), retained_dofs.len(), |row, column| {
            global_stiffness_matrix[(retained_dofs[row], retained_dofs[column])]
        })
    }

    pub fn calculate_global_load_vector(&self) -> DVector<f64>{
        let number_of_dof = self.nodes.len() * 2;
        let mut global_loads_vector = DVector::zeros(number_of_dof);

        for load in self.loads.iter(){
            let node_index = self.node_index(load.node.id) * 2;
            global_loads_vector[node_index] += load.x_force;
            global_loads_vector[node_index + 1] += load.y_force;
        }
        global_loads_vector
    }
    fn calculate_cut_global_load_vector(&self, global_load_vector: DVector<f64>) -> DVector<f64>{
        let remove_dof = self.calculate_dofs_to_cut();
        let retained_dofs: Vec<usize> = remove_dof.iter()
            .enumerate()
            .filter_map(|(index, removed)| (!removed).then_some(index))
            .collect();

        DVector::from_iterator(
            retained_dofs.len(),
            retained_dofs.iter().map(|&index| global_load_vector[index]),
        )
    }
    fn calculate_dofs_to_cut(&self) -> Vec<bool>{
        let number_of_dof = self.nodes.len() * 2;
        let mut remove_dof = vec![false; number_of_dof];

        for boundary_condition in self.boundary_conditions.iter() {
            if boundary_condition.x_fixed{
                let dof  = self.node_index(boundary_condition.node.id) * 2;
                remove_dof[dof] = true;
            }
            if boundary_condition.y_fixed{
                let dof  = self.node_index(boundary_condition.node.id) * 2 + 1;
                remove_dof[dof] = true;
            }
        }
        return remove_dof
    }
    pub fn calculate_displacements(&self, global_load_vector: DVector<f64>, global_stiffness_matrix: DMatrix<f64>) -> DVector<f64>{
        let cut_load_vector = self.calculate_cut_global_load_vector(global_load_vector);
        let cut_stiffness_matrix = self.calculate_cut_global_stiffness_matrix(global_stiffness_matrix);

        let lu_decomposition = cut_stiffness_matrix.lu();
        let reduced_displacement_solution = lu_decomposition
            .solve(&cut_load_vector)
            .expect("The reduced stiffness matrix is singular");

        let mut displacements_used = 0;
        let removed_dofs = self.calculate_dofs_to_cut();
        let num_dofs = removed_dofs.len();
        let mut displacements = DVector::from_element(num_dofs, 0.0);
        for i in 0..num_dofs{
            if removed_dofs[i]{
                continue;
            }
            displacements[i] = reduced_displacement_solution[displacements_used];
            displacements_used += 1;
        }
        return displacements
    }
    fn node_index(&self, node_id: usize) -> usize{
        for i in 0..self.nodes.len(){
            if self.nodes[i].id == node_id{
                return i;
            }
        }
        panic!("no node with that ID")
    }
}
