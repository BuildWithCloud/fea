//! Reads a truss from a JSON file. See `examples/truss.json` for the format.

use nalgebra::vector;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::error::Error;
use std::fs;
use std::path::Path;

use crate::model::boundary_condition::BoundaryCondition;
use crate::model::element::Element;
use crate::model::force::Force;
use crate::model::load::Load;
use crate::model::node::Node;
use crate::model::position::Position;
use crate::model::truss::Truss;

#[derive(Serialize, Deserialize)]
struct NodeFile {
    id: usize,
    x: f64,
    y: f64,
}

#[derive(Serialize, Deserialize)]
struct ElementFile {
    node_1: usize,
    node_2: usize,
    youngs_modulus: f64,
    area: f64,
}

#[derive(Serialize, Deserialize)]
struct LoadFile {
    node: usize,
    #[serde(default)]
    x_force: f64,
    #[serde(default)]
    y_force: f64,
}

#[derive(Serialize, Deserialize)]
struct BoundaryConditionFile {
    node: usize,
    #[serde(default)]
    x_fixed: bool,
    #[serde(default)]
    y_fixed: bool,
}

#[derive(Serialize, Deserialize)]
struct TrussFile {
    nodes: Vec<NodeFile>,
    elements: Vec<ElementFile>,
    #[serde(default)]
    loads: Vec<LoadFile>,
    #[serde(default)]
    boundary_conditions: Vec<BoundaryConditionFile>,
}

/// Loads and validates a truss from a JSON file.
pub fn read_truss(path: &Path) -> Result<Truss<2>, Box<dyn Error>> {
    truss_from_json(&fs::read_to_string(path)?)
}

/// Writes a truss (not its results) to a JSON file in the format `read_truss` accepts.
pub fn write_truss(truss: &Truss<2>, path: &Path) -> Result<(), Box<dyn Error>> {
    fs::write(path, truss_to_json(truss)?)?;
    Ok(())
}

fn truss_from_json(text: &str) -> Result<Truss<2>, Box<dyn Error>> {
    let file: TrussFile = serde_json::from_str(text)?;
    file.into_truss()
}

fn truss_to_json(truss: &Truss<2>) -> Result<String, Box<dyn Error>> {
    Ok(serde_json::to_string_pretty(&TrussFile::from_truss(truss))?)
}

impl TrussFile {
    fn from_truss(truss: &Truss<2>) -> TrussFile {
        TrussFile {
            nodes: truss.nodes.iter().map(|n| NodeFile {
                id: n.id,
                x: n.position.x,
                y: n.position.y,
            }).collect(),
            elements: truss.elements.iter().map(|e| ElementFile {
                node_1: e.node_1_id,
                node_2: e.node_2_id,
                youngs_modulus: e.youngs_modulus,
                area: e.area,
            }).collect(),
            loads: truss.loads.iter().map(|l| LoadFile {
                node: l.node_id,
                x_force: l.force.x,
                y_force: l.force.y,
            }).collect(),
            boundary_conditions: truss.boundary_conditions.iter().map(|b| BoundaryConditionFile {
                node: b.node_id,
                x_fixed: b.fixed_directions.x,
                y_fixed: b.fixed_directions.y,
            }).collect(),
        }
    }

    fn into_truss(self) -> Result<Truss<2>, Box<dyn Error>> {
        let mut ids = HashSet::new();
        for node in &self.nodes {
            if !ids.insert(node.id) {
                return Err(format!("duplicate node id {}", node.id).into());
            }
        }
        let check = |id: usize, what: &str| -> Result<(), Box<dyn Error>> {
            if ids.contains(&id) {
                Ok(())
            } else {
                Err(format!("{what} refers to non-existent node {id}").into())
            }
        };

        for element in &self.elements {
            check(element.node_1, "element")?;
            check(element.node_2, "element")?;
            if element.node_1 == element.node_2 {
                return Err(format!("element connects node {} to itself", element.node_1).into());
            }
        }
        for load in &self.loads {
            check(load.node, "load")?;
        }
        let mut constrained = HashSet::new();
        for bc in &self.boundary_conditions {
            check(bc.node, "boundary condition")?;
            if !constrained.insert(bc.node) {
                return Err(format!("node {} has more than one boundary condition", bc.node).into());
            }
        }

        let nodes: Vec<Node<2>> = self.nodes.into_iter()
            .map(|n| Node { id: n.id, position: Position(vector![n.x, n.y]) })
            .collect();

        // Zero-length elements would give an infinite stiffness.
        for element in &self.elements {
            let a = &nodes.iter().find(|n| n.id == element.node_1).unwrap().position;
            let b = &nodes.iter().find(|n| n.id == element.node_2).unwrap().position;
            if a.x == b.x && a.y == b.y {
                return Err(format!(
                    "element between nodes {} and {} has zero length", element.node_1, element.node_2
                ).into());
            }
        }

        Ok(Truss {
            nodes,
            elements: self.elements.into_iter().map(|e| Element {
                node_1_id: e.node_1,
                node_2_id: e.node_2,
                youngs_modulus: e.youngs_modulus,
                area: e.area,
            }).collect(),
            loads: self.loads.into_iter().map(|l| Load {
                node_id: l.node,
                force: Force(vector![l.x_force, l.y_force]),
            }).collect(),
            boundary_conditions: self.boundary_conditions.into_iter().map(|b| BoundaryCondition {
                node_id: b.node,
                fixed_directions: vector![b.x_fixed, b.y_fixed],
            }).collect(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_then_read_round_trips() {
        let original = truss_from_json(include_str!("../../truss_json/truss.json")).unwrap();
        let json = truss_to_json(&original).unwrap();
        let copy = truss_from_json(&json).unwrap();

        assert_eq!(truss_to_json(&copy).unwrap(), json);
        assert_eq!(copy.nodes.len(), original.nodes.len());
        assert_eq!(copy.elements.len(), original.elements.len());
        assert_eq!(copy.loads.len(), original.loads.len());
        assert_eq!(copy.boundary_conditions.len(), original.boundary_conditions.len());
    }
}
