use nalgebra::SVector;
use std::ops::Deref;
#[derive(Debug, Clone, Copy)]
pub struct Position<const D: usize>(pub SVector<f64, D>);

impl<const D: usize> Deref for Position<D>{
    type Target = SVector<f64, D>;
    fn deref(&self) -> &SVector<f64, D>{
        return &self.0;
    }
}

impl<const D: usize> std::ops::Sub for Position<D>{
    type Output = SVector<f64, D>;
    fn sub(self, rhs:Self) -> Self::Output{
        self.0 - rhs.0
    }
}