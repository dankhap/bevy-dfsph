use super::kernel3d::Kernel3D;
use crate::units::{Real, Vector3D};

/// Cubic Spline smoothing kernel.
///
/// Classic cubic spline cernel from "J. Monaghan, Smoothed Particle Hydrodynamics, “Annual Review of Astronomy and Astrophysics”, 30 (1992), pp. 543-574."
/// Normalization factors from https://pysph.readthedocs.io/en/latest/reference/kernels.html#monaghan1992
#[derive(Copy, Clone)]
pub struct CubicSpline {
    h_inv: Real,
    normalizer: Real,
    normalizer_grad: Real,

    k: Real,
    l: Real,
    radius: Real
}

impl CubicSpline {
    pub fn new(smoothing_length: Real) -> CubicSpline {
        let pi = std::f64::consts::PI as Real;
        let radius = smoothing_length;
        let h3 = radius*radius*radius;
        let h4 = h3*radius;

        CubicSpline {
            k: 8.0 / pi*h3,
            l: 48.0 / pi*h3,
            radius: radius,
            h_inv: 1.0 / smoothing_length,
            /* normalizer: 6.0 * 40.0 / (7.0 * std::f64::consts::PI as Real * smoothing_length * smoothing_length),
            normalizer_grad: 6.0 * 40.0 / (7.0 * std::f64::consts::PI as Real * smoothing_length * smoothing_length * smoothing_length), */
            /* normalizer: 6.0 * 8.0 / ( pi * smoothing_length * smoothing_length * smoothing_length ),
            normalizer_grad: 6.0 * 8.0 / (pi * smoothing_length * smoothing_length * smoothing_length * smoothing_length), */
            normalizer: 6.0 * 8.0 / ( pi * smoothing_length * smoothing_length * smoothing_length ),
            normalizer_grad: 8.0 / (pi * smoothing_length * smoothing_length * smoothing_length * smoothing_length),
        }
    }
}

impl Kernel3D for CubicSpline {
    #[inline]
    fn evaluate(&self, _r_sq: Real, r: Real) -> Real {
        let q = r * self.h_inv;
        if q <= 0.5 {
            let q2 = q * q;
            let q3 = q * q * q;
            self.k * (6.0*q3 - 6.0*q2 + 1.0)
        } else if q <= 1.0 {
            let one_minus_q = 1.0 - q;
            self.k * 2.0 * one_minus_q * one_minus_q * one_minus_q 
        } else {
            0.0
        }
    }

    #[inline]
    fn gradient(&self, ri_to_rj: Vector3D, _r_sq: Real, r: Real) -> Vector3D {
        let q = r * self.h_inv;
        if (r > 1.0e-8) && (q <= 1.0){
            let gradq = ri_to_rj * (1.0 / r * self.radius);
            if q <= 0.5 {
                self.l * (2.0 - 3.0 * q) *q * gradq
            } else {
                let factor = 1.0 - q;
                self.l * (1.0 * factor * factor)* gradq
            }
        } else {
            cgmath::Zero::zero()
        }
    }

    #[inline]
    fn laplacian(&self, _r_sq: Real, _r: Real) -> Real {
        unimplemented!();
    }
}

// generate_kernel_tests!(CubicSpline);
