use nalgebra::SVector;
pub struct BoundaryCondition<const D: usize>{
    pub node_id: usize,
    pub fixed_directions: SVector<bool, D>,
}
