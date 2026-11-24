mod model;
mod results;
mod plotting;

use model::boundary_condition::BoundaryCondition;
use model::element::Element;
use model::load::Load;
use model::node::Node;
use model::truss::Truss;
use model::position::Position;
use plotting::plot;

fn main() {
    let node1 = Node{
        id: 1,
        position: Position { x: (0.0), y: (0.0) }
    };
    let node2 = Node{
        id: 2,
        position: Position { x: (1.0), y: (0.0) }
    };
    let node3 = Node{
        id: 3,
        position: Position { x: (0.0), y: (1.0) }
    };
    let element1 = Element{
        node_1_id: node1.id,
        node_2_id: node2.id,
        youngs_modulus: 200e9,
        area: 0.001963,
    };
    let element2 = Element{
        node_1_id: node2.id,
        node_2_id: node3.id,
        youngs_modulus: 200e9,
        area: 0.001963,
    };
    let boundary_condition1 = BoundaryCondition{
        node_id: node1.id,
        x_fixed: true,
        y_fixed: true,
    };
    let boundary_condition2 = BoundaryCondition{
        node_id: node3.id,
        x_fixed: true,
        y_fixed: true,
    };
    let load1 = Load{
        node_id: node2.id,
        x_force:  0.0,
        y_force: -100.0,
    };

    let nodes = vec![node1, node2, node3];
    let elements = vec!{element1, element2};
    let boundary_conditions = vec!{boundary_condition1, boundary_condition2};
    let loads = vec![load1];
    
    
    let truss = Truss{
        nodes: nodes,
        elements: elements,
        loads: loads,
        boundary_conditions: boundary_conditions,
    };
    
    let result = truss.solve_truss();

    let displacements = &result.displacements;
    let reactions = &result.reactions;
    println!("displacements: {displacements:#?}");
    println!("reactions: {reactions:#?}");

    let path = "./plots/version1.png";
    plot::undeformed_truss(&truss, path).unwrap();

    let path =  "./plots/deformed.png";
    plot::deformed_truss(&truss, &result, 100000.0, path).unwrap();


}


