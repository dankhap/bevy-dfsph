use super::kernel::Kernel;
use crate::units::{Real, Vector};

/// Viscosity smoothing kernel.
///
/// Müller et al.'s viscosity kernel ("Particle-Based Fluid Simulation for Interactive Applications")
/// has pretty bad properties in 2D.
/// Instead, we use a Kernel proposed by Kalle Sjöström in his Master Thesis "Computational Fluid Dynamicsin 2D Game Environments"
/// (https://pdfs.semanticscholar.org/3e9c/8e0e56d4e50da62f72002a7ad3b51b742327.pdf)
#[derive(Copy, Clone)]
pub struct Viscosity {
    h: Real,
    hsq: Real,
    normalizer: Real,
    normalizer_laplacian: Real,
}

impl Viscosity {
    pub fn new(smoothing_length: Real) -> Viscosity {
        Viscosity {
            h: smoothing_length,
            hsq: smoothing_length * smoothing_length,
            normalizer: 90.0 / (29.0 * std::f64::consts::PI as Real * smoothing_length * smoothing_length),
            normalizer_laplacian: 360.0 / (29.0 * std::f64::consts::PI as Real * smoothing_length.powi(5)),
        }
    }
}

impl Kernel for Viscosity {
    #[inline]
    fn evaluate(&self, r_sq: Real, r: Real) -> Real {
        if r < self.h {
            self.normalizer * (4.0 * r_sq * r / (9.0 * self.h) + r_sq) / self.hsq
        } else {
            0.0
        }
    }

    #[inline]
    fn gradient(&self, _ri_to_rj: Vector, _r_sq: Real, _r: Real) -> Vector {
        unimplemented!();
    }

    #[inline]
    fn laplacian(&self, _r_sq: Real, r: Real) -> Real {
        self.normalizer_laplacian * (self.h - r)
    }
}

// Viscosity kernel doesn't implement gradient and integral over domain doesn't seem to be quite right.
// Wrong normalization factor?
//generate_kernel_tests!(Viscosity);
