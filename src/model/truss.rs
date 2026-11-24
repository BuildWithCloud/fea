use nalgebra::{DMatrix, DVector, Vector4, vector};

use crate::model::boundary_condition::BoundaryCondition;
use crate::model::element::{Element, ElementView};
use crate::results::element_result::ElementResult;
use crate::model::load::Load;
use crate::results::nodal_displacement::{NodalDisplacement};
use crate::model::node::{Node};
use crate::results::reaction::{Reaction};
use crate::results::truss_result::TrussResult;

pub struct Truss{
    pub nodes: Vec<Node>,
    pub elements: Vec<Element>,
    pub loads: Vec<Load>,
    pub boundary_conditions: Vec<BoundaryCondition>,
}
impl Truss{
    pub fn calculate_global_stiffness_matrix(&self) -> DMatrix<f64>{
        let number_of_dof = self.nodes.len() * 2;
        let mut k_global = DMatrix::zeros(number_of_dof, number_of_dof);

        for element in self.element_views(){
            let global_elemental_stiffness_matrix = element.stiffness_matrix();

            let node_1_index = self.node_index(element.element.node_1_id);
            let node_2_index = self.node_index(element.element.node_2_id);
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
            let node_index = self.node_index(load.node_id) * 2;
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
                let dof  = self.node_index(boundary_condition.node_id) * 2;
                remove_dof[dof] = true;
            }
            if boundary_condition.y_fixed{
                let dof  = self.node_index(boundary_condition.node_id) * 2 + 1;
                remove_dof[dof] = true;
            }
        }
        return remove_dof
    }
    fn calculate_displacements(&self, global_load_vector: DVector<f64>, global_stiffness_matrix: DMatrix<f64>) -> DVector<f64>{
        let cut_load_vector = self.calculate_cut_global_load_vector(global_load_vector);
        let cut_stiffness_matrix = self.calculate_cut_global_stiffness_matrix(global_stiffness_matrix);

        let chol = cut_stiffness_matrix.cholesky().expect("The reduced stiffness matrix is singular");
        let reduced_displacement_solution = chol.solve(&cut_load_vector);

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

    pub fn solve_truss(&self) -> TrussResult{
        let global_load_vector = self.calculate_global_load_vector();
        let global_stiffness_matrix = self.calculate_global_stiffness_matrix();

        let displacements = self.calculate_displacements(global_load_vector, global_stiffness_matrix);

        let mut element_results: Vec<ElementResult> = vec![];

        for element in self.element_views(){
            let node_1_index = self.node_index(element.element.node_1_id);
            let node_2_index = self.node_index(element.element.node_2_id);
            let dof = [node_1_index * 2, node_1_index * 2 + 1, node_2_index * 2, node_2_index * 2 + 1];
            let global_displacement:Vector4<f64> = vector![displacements[dof[0]],displacements[dof[0]+1],displacements[dof[2]], displacements[dof[2]+1]];
            

            let element_result = ElementResult{
                node_1_id: element.element.node_1_id,
                node_2_id: element.element.node_2_id,
                theta: element.theta(),
                axial_force: element.axial_force(&global_displacement),
                stress: element.stress(&global_displacement),
                strain: element.strain(&global_displacement)
            };
            element_results.push(element_result);
        }

        let mut reactions: Vec<Reaction> = vec![];
        for boundary_condition in &self.boundary_conditions{
            let mut force_x = 0.0;
            let mut force_y = 0.0;
            for element_result in &element_results{
                if element_result.node_1_id == boundary_condition.node_id{
                    force_x -= element_result.axial_force * element_result.theta.cos();
                    force_y -= element_result.axial_force * element_result.theta.sin();
                }
                else if element_result.node_2_id == boundary_condition.node_id {
                    force_x += element_result.axial_force * element_result.theta.cos();
                    force_y += element_result.axial_force * element_result.theta.sin();
                }
            }
            let reaction = Reaction{
                node_id: boundary_condition.node_id,
                x_force: force_x,
                y_force: force_y
            };
            reactions.push(reaction);
        }

        let mut nodal_displacements: Vec<NodalDisplacement> = vec![];
        for node in &self.nodes{
            let dof = self.node_index(node.id) * 2;
            let nodal_displacement = NodalDisplacement{
                node_id: node.id,
                x_displacement: displacements[dof],
                y_displacement: displacements[dof+1],
            };
            nodal_displacements.push(nodal_displacement);
        }


        return TrussResult{
            displacements: nodal_displacements,
            reactions: reactions,
            element_results: element_results,
        }

    }

    /// Pairs each element with the nodes it connects.
    pub fn element_views(&self) -> impl Iterator<Item = ElementView<'_>>{
        self.elements.iter().map(|element| ElementView{
            element,
            node_1: self.node(element.node_1_id),
            node_2: self.node(element.node_2_id),
        })
    }

    pub fn node(&self, node_id: usize) -> &Node{
        &self.nodes[self.node_index(node_id)]
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
