use nalgebra::SVector;
use std::ops::Deref;
#[derive(Debug)]
pub struct Position<const D: usize>(pub SVector<f64, D>);

impl<const D: usize> Deref for Position<D>{
    type Target = SVector<f64, D>;
    fn deref(&self) -> &SVector<f64, D>{
        return &self.0;
    }
}