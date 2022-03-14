pub use dfsph3d::DFSPHSolver3D;
// pub use wscsph::WCSPHSolver;

mod dfsph3d;
// mod wscsph;

// ------------------------------------------------------

use super::fluidparticleworld_3d::FluidParticleWorld3D;
use super::timemanager::TimeManager;

/* pub trait Solver {
    // todo: this is not elegant, should be done automatically
    fn clear_cached_data(&mut self);

    // performs a single simulation step.
    fn simulation_step(&mut self, fluid_world: &mut FluidParticleWorld, time_manager: &mut TimeManager);
} */


pub trait Solver3D {
    // todo: this is not elegant, should be done automatically
    fn clear_cached_data(&mut self);

    // performs a single simulation step.
    fn simulation_step(&mut self, fluid_world: &mut FluidParticleWorld3D, time_manager: &mut TimeManager);
}
