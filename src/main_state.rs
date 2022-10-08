use crate::units::*;

use bevy::render::mesh::{Indices, Mesh};
use bevy::render::pipeline::PrimitiveTopology;
use std::collections::VecDeque;
use std::time::{Duration, Instant};

use std::sync::{Arc, Mutex};
use super::sph;
use std::cmp::Ordering;
const CAMERA_SCALE: f32 = 300.0;


const SIMULATION_STEP_HISTORY_LENGTH: usize = 80;

const TARGET_FPS: Real = 60.0;
// Simulation time will try to stay sync with real time unless framerate drops below this. If it simulates slower, simulation time slows down.
// Note that this measure avoid the "well of despair" (as dubbed by https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/BestPractices.html)
// where we need to do more physics step to catch up, but by doing so take even longer to catch up.
const TARGET_FPS_MIN: Real = 10.0;
const TARGET_MAX_PROCESSING_TIME: Real = 1.0 / TARGET_FPS_MIN;

// Desired relationship between time in reality and time in simulation. In other word, "speed factor"
// (that is, if we simulation processing time is low enough, otherwise simulation will slow down regardless)
const REALTIME_TO_SIMTIME_SCALE: f32 = 1.0;

const TARGET_FRAME_SIMDURATION: Real = REALTIME_TO_SIMTIME_SCALE / TARGET_FPS;

pub struct MainState {
    pub fluid_world: sph::FluidParticleWorld3D,
    time_manager: sph::TimeManager,
    sph_solver: Arc<Mutex<dyn sph::Solver3D + Send>>,

    simulation_step_duration_history: VecDeque<Duration>,
    simulation_processing_time_frame: Duration,
    simulationstep_count_frame: u32,

    simulation_starttime: Instant,
    simulation_processing_time_total: Duration,
    simulation_to_realtime_offset: f32, // Starts out with 0 and grows if we spend too much time on processing the simulation

    frame_counter: usize,

}


pub struct Box {
    pub min_x: f32,
    pub max_x: f32,

    pub min_y: f32,
    pub max_y: f32,

    pub min_z: f32,
    pub max_z: f32,
}

impl Box {
    pub fn new(x_length: f32, y_length: f32, z_length: f32) -> Box {
        Box {
            max_x: x_length / 2.0,
            min_x: -x_length / 2.0,
            max_y: y_length / 2.0,
            min_y: -y_length / 2.0,
            max_z: z_length / 2.0,
            min_z: -z_length / 2.0,
        }
    }
}

impl Default for Box {
    fn default() -> Self {
        Box::new(1.0, 1.0, 1.0)
    }
}

impl MainState {
    pub fn new(smoothing_length: f32, particles_num: f32, density: f32) -> MainState {

        let mut fluid_world = sph::FluidParticleWorld3D::new(
            smoothing_length,    // smoothing factor
            particles_num, // #1660, 5000 particles/m²
            density,  // density of water (? this is 2d, not 3d where it's 1000 kg/m³)
        );
        let xsph = sph::XSPHViscosityModel3D::new(fluid_world.properties.smoothing_length());
        //xsph.epsilon = 0.1;
        /* let mut physicalviscosity = 
            sph::PhysicalViscosityModel::new(fluid_world.properties.smoothing_length());
        physicalviscosity.fluid_viscosity = 0.01; */

        let sph_solver: Arc<Mutex<dyn sph::Solver3D + Send> > = 
            Arc::new(Mutex::new(sph::DFSPHSolver3D::new(xsph, fluid_world.properties.smoothing_length())));


        // let main_state = MainState::new)
        let mut simulator = 0;
        Self::reset_fluid(&mut fluid_world);

        let cfl_factor = 1.5;
        let time_manager = sph::TimeManager::new(
            sph::TimeManagerConfiguration::FixedTimeStep(TARGET_FRAME_SIMDURATION / 20.0));
            /* sph::TimeManagerConfiguration::AdaptiveTimeStep {
                timestep_max: TARGET_FRAME_SIMDURATION / 4.0,
                timestep_min: REALTIME_TO_SIMTIME_SCALE / (400.0 * 60.0), // Don't do steps that results in more than a 400 steps for an image on a classic 60Hz display
                timestep_target_frame: sph::AdaptiveTimeStepTarget::None,
                cfl_factor,
            },
        ); */

        MainState {
            // update_mode: UpdateMode::RealTime,
            fluid_world,
            time_manager,
            sph_solver,

            // camera: Camera::center_around_world_rect(graphics::screen_coordinates(ctx), Rect::new(-0.1, -0.1, 2.1, 1.6)),

            simulation_step_duration_history: VecDeque::with_capacity(SIMULATION_STEP_HISTORY_LENGTH),
            simulation_processing_time_frame: Default::default(),
            simulationstep_count_frame: 0,

            simulation_starttime: Instant::now(),
            simulation_processing_time_total: Default::default(),
            simulation_to_realtime_offset: Default::default(),

            frame_counter: 0,
        }
    }

    pub fn get_fluid_cm(&self) -> Point3D{
        let mut tx = 0.0;
        let mut ty = 0.0;
        let mut tz = 0.0;
        let mut total = 0.0;
        
        for p in self.fluid_world.particles.positions.iter(){
            let pos = Point3D::new(p.x , p.y, p.z );
            tx += p.x;
            ty += p.y;
            tz += p.z;
            total += 1.0;
        }

        Point3D::new(tx/total, ty/total, tz/total)
    }

    pub fn get_fluid_bbox(&self) -> Box{
        
        let mut x_pos = Vec::<Real>::with_capacity(self.fluid_world.particles.positions.len());
        let mut y_pos = Vec::<Real>::with_capacity(self.fluid_world.particles.positions.len());
        let mut z_pos = Vec::<Real>::with_capacity(self.fluid_world.particles.positions.len());
        for p in self.fluid_world.particles.positions.iter(){
            x_pos.push(p.x);
            y_pos.push(p.y);
            z_pos.push(p.z);
        }
        x_pos.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        y_pos.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        z_pos.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        let sp = Box{min_x: x_pos[0], 
                     max_x: x_pos[x_pos.len()-1], 
                     min_y: y_pos[0], 
                     max_y: y_pos[y_pos.len()-1], 
                     min_z: z_pos[0], 
                     max_z: z_pos[z_pos.len()-1]};
        sp
    }

    pub fn get_vertices(&self, lbox: Box) -> Vec<[f32; 3]>{

        // let sp = Box{min_x: -lsize, max_x: lsize, min_y: -lsize, max_y: lsize, min_z: -lsize, max_z: lsize} ;
        let sp = lbox;
          let vertices = &[
                // Top
                ([sp.min_x, sp.min_y, sp.max_z], [0., 0., 1.0], [0., 0.]),
                ([sp.max_x, sp.min_y, sp.max_z], [0., 0., 1.0], [1.0, 0.]),
                ([sp.max_x, sp.max_y, sp.max_z], [0., 0., 1.0], [1.0, 1.0]),
                ([sp.min_x, sp.max_y, sp.max_z], [0., 0., 1.0], [0., 1.0]),
                // Bottom
                ([sp.min_x, sp.max_y, sp.min_z], [0., 0., -1.0], [1.0, 0.]),
                ([sp.max_x, sp.max_y, sp.min_z], [0., 0., -1.0], [0., 0.]),
                ([sp.max_x, sp.min_y, sp.min_z], [0., 0., -1.0], [0., 1.0]),
                ([sp.min_x, sp.min_y, sp.min_z], [0., 0., -1.0], [1.0, 1.0]),
                // Right
                ([sp.max_x, sp.min_y, sp.min_z], [1.0, 0., 0.], [0., 0.]),
                ([sp.max_x, sp.max_y, sp.min_z], [1.0, 0., 0.], [1.0, 0.]),
                ([sp.max_x, sp.max_y, sp.max_z], [1.0, 0., 0.], [1.0, 1.0]),
                ([sp.max_x, sp.min_y, sp.max_z], [1.0, 0., 0.], [0., 1.0]),
                // Left
                ([sp.min_x, sp.min_y, sp.max_z], [-1.0, 0., 0.], [1.0, 0.]),
                ([sp.min_x, sp.max_y, sp.max_z], [-1.0, 0., 0.], [0., 0.]),
                ([sp.min_x, sp.max_y, sp.min_z], [-1.0, 0., 0.], [0., 1.0]),
                ([sp.min_x, sp.min_y, sp.min_z], [-1.0, 0., 0.], [1.0, 1.0]),
                // Front
                ([sp.max_x, sp.max_y, sp.min_z], [0., 1.0, 0.], [1.0, 0.]),
                ([sp.min_x, sp.max_y, sp.min_z], [0., 1.0, 0.], [0., 0.]),
                ([sp.min_x, sp.max_y, sp.max_z], [0., 1.0, 0.], [0., 1.0]),
                ([sp.max_x, sp.max_y, sp.max_z], [0., 1.0, 0.], [1.0, 1.0]),
                // Back
                ([sp.max_x, sp.min_y, sp.max_z], [0., -1.0, 0.], [0., 0.]),
                ([sp.min_x, sp.min_y, sp.max_z], [0., -1.0, 0.], [1.0, 0.]),
                ([sp.min_x, sp.min_y, sp.min_z], [0., -1.0, 0.], [1.0, 1.0]),
                ([sp.max_x, sp.min_y, sp.min_z], [0., -1.0, 0.], [0., 1.0]),
            ];

            let mut positions = Vec::with_capacity(24);

            for (position, normal, uv) in vertices.iter() {
                positions.push(*position);
            }
            positions

    }
    pub fn get_mesh(&self, lsize: f32) -> Mesh {
        // let sp: Box = Default::default();
        let sp = Box{min_x: -lsize, max_x: lsize, min_y: -lsize, max_y: lsize, min_z: -lsize, max_z: lsize} ;
          let vertices = &[
                // Top
                ([sp.min_x, sp.min_y, sp.max_z], [0., 0., 1.0], [0., 0.]),
                ([sp.max_x, sp.min_y, sp.max_z], [0., 0., 1.0], [1.0, 0.]),
                ([sp.max_x, sp.max_y, sp.max_z], [0., 0., 1.0], [1.0, 1.0]),
                ([sp.min_x, sp.max_y, sp.max_z], [0., 0., 1.0], [0., 1.0]),
                // Bottom
                ([sp.min_x, sp.max_y, sp.min_z], [0., 0., -1.0], [1.0, 0.]),
                ([sp.max_x, sp.max_y, sp.min_z], [0., 0., -1.0], [0., 0.]),
                ([sp.max_x, sp.min_y, sp.min_z], [0., 0., -1.0], [0., 1.0]),
                ([sp.min_x, sp.min_y, sp.min_z], [0., 0., -1.0], [1.0, 1.0]),
                // Right
                ([sp.max_x, sp.min_y, sp.min_z], [1.0, 0., 0.], [0., 0.]),
                ([sp.max_x, sp.max_y, sp.min_z], [1.0, 0., 0.], [1.0, 0.]),
                ([sp.max_x, sp.max_y, sp.max_z], [1.0, 0., 0.], [1.0, 1.0]),
                ([sp.max_x, sp.min_y, sp.max_z], [1.0, 0., 0.], [0., 1.0]),
                // Left
                ([sp.min_x, sp.min_y, sp.max_z], [-1.0, 0., 0.], [1.0, 0.]),
                ([sp.min_x, sp.max_y, sp.max_z], [-1.0, 0., 0.], [0., 0.]),
                ([sp.min_x, sp.max_y, sp.min_z], [-1.0, 0., 0.], [0., 1.0]),
                ([sp.min_x, sp.min_y, sp.min_z], [-1.0, 0., 0.], [1.0, 1.0]),
                // Front
                ([sp.max_x, sp.max_y, sp.min_z], [0., 1.0, 0.], [1.0, 0.]),
                ([sp.min_x, sp.max_y, sp.min_z], [0., 1.0, 0.], [0., 0.]),
                ([sp.min_x, sp.max_y, sp.max_z], [0., 1.0, 0.], [0., 1.0]),
                ([sp.max_x, sp.max_y, sp.max_z], [0., 1.0, 0.], [1.0, 1.0]),
                // Back
                ([sp.max_x, sp.min_y, sp.max_z], [0., -1.0, 0.], [0., 0.]),
                ([sp.min_x, sp.min_y, sp.max_z], [0., -1.0, 0.], [1.0, 0.]),
                ([sp.min_x, sp.min_y, sp.min_z], [0., -1.0, 0.], [1.0, 1.0]),
                ([sp.max_x, sp.min_y, sp.min_z], [0., -1.0, 0.], [0., 1.0]),
            ];

            let mut positions = Vec::with_capacity(24);
            let mut normals = Vec::with_capacity(24);
            let mut uvs = Vec::with_capacity(24);

            for (position, normal, uv) in vertices.iter() {
                positions.push(*position);
                normals.push(*normal);
                uvs.push(*uv);
            }

            let indices = Indices::U32(vec![
                0, 1, 2, 2, 3, 0, // top
                4, 5, 6, 6, 7, 4, // bottom
                8, 9, 10, 10, 11, 8, // right
                12, 13, 14, 14, 15, 12, // left
                16, 17, 18, 18, 19, 16, // front
                20, 21, 22, 22, 23, 20, // back
            ]);

            let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
            mesh.set_attribute(Mesh::ATTRIBUTE_POSITION, positions);
            mesh.set_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
            mesh.set_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
            mesh.set_indices(Some(indices));
            mesh
    }

    pub fn get_particale_radius(&self) -> f32 {
        let particle_radius = self.fluid_world.properties.particle_radius()*CAMERA_SCALE;
        return particle_radius;
    }

    pub fn get_particle_num(&self) -> usize {
        return self.fluid_world.particles.positions.len();
    }

    fn reset_fluid(fluid_world: &mut sph::FluidParticleWorld3D) {
        fluid_world.remove_all_fluid_particles();
        fluid_world.remove_all_boundary_particles();

        // fluid_world.add_fluid_rect(&Rect3D::new(0.3, 0.0, -0.1, 0.3, 0.3, 0.3), 0.05);
        fluid_world.cube_fluid(Vector3D::new(0.0, 1.5, 0.0), 15, 15, 15, 0.05);
        // fluid_world.cube_fluid(Vector3D::new(0.0, 0.0, 0.0), 7, 7, 7, 0.05);

        fluid_world.add_boundary_rect(&Rect3D::new(-2.5, -1.2, -2.5, 5.0, 0.4, 5.0)); //bottom
        // fluid_world.add_boundary_rect(&Rect3D::new(-0.1,  1.5, -0.5, 1.0, 0.1, 1.0)); //top

        /* fluid_world.add_boundary_rect(&Rect3D::new(-2.5, -1.5, -2.5, 5.0, 1.4, 0.2));
        fluid_world.add_boundary_rect(&Rect3D::new(-2.5, -1.5, -2.5, 0.2, 1.4, 5.0));
        fluid_world.add_boundary_rect(&Rect3D::new(-2.5, -1.5, 2.3, 5.0, 1.4, 0.2));
        fluid_world.add_boundary_rect(&Rect3D::new(2.3, -1.5, -2.5, 0.2, 1.4, 5.0)); */
    }

    pub fn single_sim_step(&mut self) {
        let time_before = Instant::now();
        self.sph_solver.lock().unwrap().simulation_step(&mut self.fluid_world, &mut self.time_manager);
        let time_after = Instant::now();

        let step_processing_time = time_after - time_before;
        self.simulation_processing_time_frame += step_processing_time;
        self.simulation_processing_time_total += step_processing_time;
        self.simulationstep_count_frame += 1;

        if self.simulation_step_duration_history.len() == SIMULATION_STEP_HISTORY_LENGTH {
            self.simulation_step_duration_history.pop_front();
        }
        self.simulation_step_duration_history.push_back(step_processing_time);
    }

    pub fn reset_simulation(&mut self) {
        self.sph_solver.lock().unwrap().clear_cached_data(); // todo: this is super meh
        self.simulation_starttime = Instant::now();
        self.simulation_to_realtime_offset = 0.0;
        self.simulation_processing_time_total = Default::default();

        self.frame_counter = 0;
        self.time_manager.restart();
        Self::reset_fluid(&mut self.fluid_world);
    }
}
