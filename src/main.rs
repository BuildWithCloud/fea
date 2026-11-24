mod boundary_condition;
mod element;
mod load;
mod node;
mod truss;
mod truss_result;
mod nodal_displacement;
mod element_result;
mod reaction;

use boundary_condition::BoundaryCondition;
use element::Element;
use load::Load;
use node::Node;
use truss::Truss;

fn main() {
    let node1 = Node{
        id: 1,
        x_position: 0.0,
        y_position: 0.0,
    };
    let node2 = Node{
        id: 2,
        x_position: 1.0,
        y_position: 0.0,
    };
    let node3 = Node{
        id: 3,
        x_position: 0.0,
        y_position: 1.0,
    };
    let element1 = Element{
        node_1: &node1,
        node_2: &node2,
        youngs_modulus: 200e9,
        area: 0.001963,
    };
    let element2 = Element{
        node_1: &node2,
        node_2: &node3,
        youngs_modulus: 200e9,
        area: 0.001963,
    };
    let boundary_condition1 = BoundaryCondition{
        node: &node1,
        x_fixed: true,
        y_fixed: true,
    };
    let boundary_condition2 = BoundaryCondition{
        node: &node3,
        x_fixed: true,
        y_fixed: true,
    };
    let load1 = Load{
        node: &node2,
        x_force:  0.0,
        y_force: -100.0,
    };

    let nodes = vec![&node1, &node2, &node3];
    let elements = vec!{&element1, &element2};
    let boundary_conditions = vec!{&boundary_condition1, &boundary_condition2};
    let loads = vec![&load1];
    
    
    let truss = Truss{
        nodes: nodes,
        elements: elements,
        loads: loads,
        boundary_conditions: boundary_conditions,
    };
    
    let result = truss.solve_truss();

    let displacements = result.displacements;
    let reactions = result.reactions;
    println!("displacements: {displacements:#?}");
    println!("reactions: {reactions:#?}");


}


