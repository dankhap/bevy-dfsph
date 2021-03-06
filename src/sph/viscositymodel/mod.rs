// pub use physical::PhysicalViscosityModel;
pub use xsph3d::XSPHViscosityModel3D;

// mod physical;
mod xsph3d;

// ------------------------------------------------------

use crate::units::{Real, Vector3D};

pub trait ViscosityModel {
    // computes viscious accelleration for a particle i
    //
    // todo. integrating like this seems to be tricky! that's a ton of parameters that might be unused!
    // sphlishsphlash is just reiterating on all particles instead for the viscosity model
    // maybe set some of them and store model specific factor.
    fn compute_viscous_accelleration(&self, dt: Real, r_sq: Real, r: Real, massj: Real, rhoj: Real, velocitydiff: Vector3D) -> Vector3D;
}


