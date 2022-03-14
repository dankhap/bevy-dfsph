use crate::units::*;
use cgmath::prelude::*;
use rand::{Rng, SeedableRng};
use rand::rngs::SmallRng;
use rayon::prelude::*;

use super::neighborhood_search_3d::{NeighborhoodSearch, ParticleIndex};
use super::scratch_buffer_3d::ScratchBufferStore;
use super::smoothing_kernel::Kernel3D;

pub struct Particles {
    pub positions: Vec<Point3D>,
    pub velocities: Vec<Vector3D>,

    // Local densities ρ
    // typically recomputed every frame
    pub densities: Vec<Real>,

    // also called "shadow particles", immovable particles used for boundaries
    pub boundary_particles: Vec<Point3D>,

    neighborhood: NeighborhoodSearch,
}

impl Particles {
    #[inline(always)]
    pub(super) fn foreach_neighbor_particle(&self, pidx: ParticleIndex, f: impl FnMut(ParticleIndex) -> ()) {
        Self::foreach_neighbor_particle_internal(&self.neighborhood, pidx, f)
    }
    #[inline]
    fn foreach_neighbor_particle_internal(neighborhood: &NeighborhoodSearch, pidx: ParticleIndex, f: impl FnMut(ParticleIndex) -> ()) {
        neighborhood.foreach_neighbor(pidx, f);
    }

    #[inline(always)]
    pub(super) fn foreach_neighbor_particle_boundary(&self, pidx: ParticleIndex, f: impl FnMut(ParticleIndex) -> ()) {
        Self::foreach_neighbor_particle_internal_boundary_new(&self.neighborhood, pidx, f)
    }
    #[inline]
    fn foreach_neighbor_particle_internal_boundary_new(neighborhood: &NeighborhoodSearch, pidx: ParticleIndex, f: impl FnMut(ParticleIndex) -> ()) {
        neighborhood.foreach_boundary_neighbor(pidx, f);
    }

    // Can be useful to determine particle deficiency. Not used right now.
    #[inline]
    pub(super) fn num_total_neighbors(&self, pidx: ParticleIndex) -> u64 {
        self.neighborhood.num_neighbors(pidx) + self.neighborhood.num_boundary_neighbors(pidx)
    }
}

pub struct ConstantFluidProperties {
    smoothing_length: Real, // typically expressed as 'h'
    particle_density: Real, // #particles/m² for resting fluid
    fluid_density: Real,    // kg/m² for the resting fluid (ρ, rho)
}

impl ConstantFluidProperties {
    fn new(
        smoothing_factor: Real,
        particle_density: Real, // #particles/m² for resting fluid
        fluid_density: Real,    // kg/m² for the resting fluid) {
    ) -> ConstantFluidProperties {
        let smoothing_length = 2.0 * Self::particle_radius_from_particle_density(particle_density) * smoothing_factor;
        ConstantFluidProperties {
            smoothing_length,
            particle_density,
            fluid_density,
        }
    }

    pub fn smoothing_length(&self) -> Real {
        self.smoothing_length
    }

    pub fn fluid_density(&self) -> Real {
        self.fluid_density
    }

    pub fn particle_mass(&self) -> Real {
        self.fluid_density / self.particle_density
    }

    fn num_particles_per_meter(&self) -> Real {
        self.particle_density.sqrt()
    }

    fn particle_radius_from_particle_density(particle_density: Real) -> Real {
        // density is per m²
        0.5 / particle_density.sqrt()
    }

    pub fn particle_radius(&self) -> Real {
        Self::particle_radius_from_particle_density(self.particle_density)
    }
}

pub struct FluidParticleWorld3D {
    pub particles: Particles,
    pub properties: ConstantFluidProperties,

    pub(super) scratch_buffers: ScratchBufferStore,

    pub gravity: Vector3D, // global gravity force in m/s² (== N/kg)

    // tracks whether boundary particles have been added/moved
    boundary_changed: bool,
}
impl FluidParticleWorld3D {
    pub fn new(
        smoothing_factor: Real,
        particle_density: Real, // #particles/m² for resting fluid
        fluid_density: Real,    // kg/m² for the resting fluid
    ) -> FluidParticleWorld3D {
        let properties = ConstantFluidProperties::new(smoothing_factor, particle_density, fluid_density);
        FluidParticleWorld3D {
            particles: Particles {
                positions: Vec::new(),
                velocities: Vec::new(),
                densities: Vec::new(),

                boundary_particles: Vec::new(),

                neighborhood: NeighborhoodSearch::new(properties.smoothing_length()),
            },
            properties,
            scratch_buffers: ScratchBufferStore::new(),

            gravity: Vector3D::new(0.0, -9.81, 0.0),

            boundary_changed: true,
        }
    }

    pub fn remove_all_fluid_particles(&mut self) {
        self.particles.positions.clear();
        self.particles.velocities.clear();
    }

    pub fn remove_all_boundary_particles(&mut self) {
        self.particles.boundary_particles.clear();
        self.particles.velocities.clear();
    }

    /// - `jitter`: Amount of jitter. 0 for perfect lattice. >1 and particles are no longer in a strict lattice.
    pub fn add_fluid_rect(&mut self, fluid_rect: &Rect3D, jitter_amount: Real) {
        // fluid_rect.w * fluid_rect.h / self.particle_density, but discretized per axis
        let num_particles_per_meter = self.properties.num_particles_per_meter();
        let num_particles_x = std::cmp::max(1, (fluid_rect.w as Real * num_particles_per_meter) as usize);
        let num_particles_y = std::cmp::max(1, (fluid_rect.h as Real * num_particles_per_meter) as usize);
        let num_particles_z = std::cmp::max(1, (fluid_rect.d as Real * num_particles_per_meter) as usize);
        let num_particles = num_particles_x * num_particles_y * num_particles_z;

        let new_total_particle_count = self.particles.positions.len() + num_particles;
        self.particles.positions.reserve(new_total_particle_count);
        self.particles.velocities.resize(new_total_particle_count, Zero::zero());
        self.particles.densities.resize(new_total_particle_count, Zero::zero());

        let mut rng: SmallRng = SeedableRng::seed_from_u64(self.particles.positions.len() as u64);

        let bottom_left = Point3D::new(fluid_rect.x as Real, fluid_rect.y as Real, fluid_rect.z as Real);
        let step = (fluid_rect.w as Real / (num_particles_x as Real)).min(fluid_rect.h as Real / (num_particles_y as Real));
        let jitter_factor = step * jitter_amount;
        for y in 0..num_particles_y {
            for x in 0..num_particles_x {
                for z in 0..num_particles_z {
                    let jitter = (rng.gen::<Vector3D>() * 0.5 + Vector3D::new(0.5, 0.5, 0.5)) * jitter_factor;
                    self.particles
                        .positions
                        .push(bottom_left + jitter + Vector3D::new(step * (x as Real), step * (y as Real), step * (z as Real)));
                    
                }
                
            }
            
        }
    }


    pub fn add_boundary_thick_plane(&mut self, start: Point3D, end: Point3D, thickness_in_particles: u32) {
        let dir = (end - start).normalize();
        let dir_perpendicular = Vector3D::new(-dir.y, dir.x, dir.z);
        let thickness_world = thickness_in_particles as Real / self.properties.num_particles_per_meter();
        let elongation = dir * thickness_world;
        let mut offset = -dir_perpendicular * thickness_world;
        let step = dir_perpendicular * thickness_world / thickness_in_particles as Real;
        for _ in 0..thickness_in_particles {
            self.add_boundary_line(start + offset, end + offset + elongation);
            offset += step;
        }
    }

    pub fn add_boundary_thick_line(&mut self, start: Point3D, end: Point3D, thickness_in_particles: u32) {
        let dir = (end - start).normalize();
        let dir_perpendicular = Vector3D::new(-dir.y, dir.x, dir.z);
        let thickness_world = thickness_in_particles as Real / self.properties.num_particles_per_meter();
        let elongation = dir * thickness_world;
        let mut offset = -dir_perpendicular * thickness_world;
        let step = dir_perpendicular * thickness_world / thickness_in_particles as Real;
        for _ in 0..thickness_in_particles {
            self.add_boundary_line(start + offset, end + offset + elongation);
            offset += step;
        }
    }

    pub fn add_boundary_line(&mut self, start: Point3D, end: Point3D) {
        let distance = start.distance(end);
        let num_particles_per_meter = self.properties.num_particles_per_meter();
        let num_shadow_particles = std::cmp::max(1, (distance * num_particles_per_meter).ceil() as usize);
        self.particles.boundary_particles.reserve(num_shadow_particles);
        let step = (end - start) / distance / self.properties.num_particles_per_meter();

        let mut pos = start; //- step * 0.5;
        for _ in 0..num_shadow_particles {
            self.particles.boundary_particles.push(pos);
            pos += step;
        }

        self.boundary_changed = true;
    }

    pub(super) fn update_densities(&mut self, kernel: impl Kernel3D + std::marker::Sync) {
        microprofile::scope!("FluidParticleWorld3D", "update_densities");
        assert_eq!(self.particles.positions.len(), self.particles.densities.len());

        let mass = self.properties.particle_mass();
        let fluid_density = self.properties.fluid_density;
        let neighborhood = &self.particles.neighborhood;
        let positions = &self.particles.positions;
        let boundary_positions = &self.particles.boundary_particles;
        println!("boundary_positions: {0}", boundary_positions.len());
        self.particles
            .densities
            .par_iter_mut()
            .zip(positions.par_iter())
            .enumerate()
            .for_each(|(i, (density, ri))| {
                *density = kernel.evaluate(0.0, 0.0) * mass; // self-contribution
                let i = i as u64;
                Particles::foreach_neighbor_particle_internal(
                    &neighborhood,
                    i,
                    #[inline(always)]
                    |j| {
                        let r_sq = ri.distance2(unsafe { *positions.get_unchecked(j as usize) });
                        let density_contribution = kernel.evaluate(r_sq, r_sq.sqrt()) * mass;
                        *density += density_contribution;
                    },
                );
                Particles::foreach_neighbor_particle_internal_boundary_new(
                    &neighborhood,
                    i,
                    #[inline(always)]
                    |j| {
                        println!("processing boundary neighborhood {0}", j);
                        let r_sq = ri.distance2(unsafe { *boundary_positions.get_unchecked(j as usize) });
                        let density_contribution = kernel.evaluate(r_sq, r_sq.sqrt()) * mass;
                        *density += density_contribution;
                    },
                );

                // Pressure clamping to work around particle deficiency problem. Good explanation here:
                // https://github.com/InteractiveComputerGraphics/SPlisHSPlasH/issues/36#issuecomment-495883932
                *density = density.max(fluid_density);
            });
        println!("finish up density");
    }

    // sorts particle attributes internally!
    // TODO: put on particles struct
    pub(super) fn update_neighborhood_datastructure<'a>(
        &'a mut self,
        additional_particle_attributes_vector: Vec<&'a mut Vec<Vector3D>>,
        additional_particle_attributes_real: Vec<&'a mut Vec<Real>>,
    ) {
        microprofile::scope!("FluidParticleWorld3D", "update_neighborhood_datastructure");

        let mut additional_particle_attributes_vector = additional_particle_attributes_vector;
        additional_particle_attributes_vector.push(&mut self.particles.velocities);

        let mut additional_particle_attributes_real = additional_particle_attributes_real;

        if self.boundary_changed {
            self.particles
                .neighborhood
                .update_boundary(&mut self.scratch_buffers, &mut self.particles.boundary_particles);
            self.boundary_changed = false;
        }

        self.particles.neighborhood.update_particle_neighbors(
            &mut self.scratch_buffers,
            &mut self.particles.positions,
            &mut additional_particle_attributes_vector,
            &mut additional_particle_attributes_real,
            &self.particles.boundary_particles,
        );
    }
}
