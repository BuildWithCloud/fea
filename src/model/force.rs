use nalgebra::SVector;
use std::ops::Deref;

#[derive(Debug)]
pub struct Force<const D: usize>(pub SVector<f64, D>);

impl<const D: usize> Deref for Force<D>{
    type Target = SVector<f64, D>;
    fn deref(&self) -> &SVector<f64, D>{
        return &self.0;
    }
}