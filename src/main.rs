use bevy::prelude::*;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
// use bevy_prototype_lyon::prelude::*;
// use cgmath::prelude::*;
use std::collections::VecDeque;
use std::time::{Duration, Instant};
mod camera;

use std::sync::{Arc, Mutex};
use bevysph::sph;
use bevysph::units::{Point, Real, Rect3D, Point3D};

use rand::{prelude::SliceRandom, Rng};
use std::{
    collections::BTreeSet,
};
use bevy::render::camera::{Camera, CameraProjection, DepthCalculation, VisibleEntities};


use itertools::linspace;

struct SimpleOrthoProjection {
    far: f32,
    aspect: f32,
}

impl CameraProjection for SimpleOrthoProjection {
    fn get_projection_matrix(&self) -> Mat4 {
        Mat4::orthographic_rh(
            -self.aspect, self.aspect, 0.1, 3.0, 0.0, self.far
        )
    }

    // what to do on window resize
    fn update(&mut self, width: f32, height: f32) {
        self.aspect = width / height;
    }

    fn depth_calculation(&self) -> DepthCalculation {
        // for 2D (camera doesn't rotate)
        DepthCalculation::ZDifference

        // otherwise
        // DepthCalculation::Distance
    }
}

impl Default for SimpleOrthoProjection {
    fn default() -> Self {
        Self { far: 1000.0, aspect: 1.0 }
    }
}

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

struct Particle;
struct Velocity {
    translation: Vec3,
    rotation: f32,
}
struct ParticlePool {
    particle_grid: Vec3,
}
struct Position(Vec3);
struct Name(String);
struct GreetTimer(Timer);
type Particles = BTreeSet<String>;
struct SelectTimer;
struct ContributorDisplay;
struct ContributorSelection {
    order: Vec<(String, Entity)>,
    idx: usize,
}

struct MainState {
    fluid_world: sph::FluidParticleWorld3D,
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

impl MainState {
    pub fn new() -> MainState {

        let mut fluid_world = sph::FluidParticleWorld3D::new(
            2.0,    // smoothing factor
            1000.0, // #1660, 5000 particles/m²
            1000.0,  // density of water (? this is 2d, not 3d where it's 1000 kg/m³)
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

        let cfl_factor = 1.0;
        let time_manager = sph::TimeManager::new(
            //sph::TimeManagerConfiguration::FixedTimeStep(TARGET_FRAME_SIMDURATION / 20.0));
            sph::TimeManagerConfiguration::AdaptiveTimeStep {
                timestep_max: TARGET_FRAME_SIMDURATION / 4.0,
                timestep_min: REALTIME_TO_SIMTIME_SCALE / (400.0 * 60.0), // Don't do steps that results in more than a 400 steps for an image on a classic 60Hz display
                timestep_target_frame: sph::AdaptiveTimeStepTarget::None,
                cfl_factor,
            },
        );

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
    fn get_particale_radius(&self) -> f32 {
        let particle_radius = self.fluid_world.properties.particle_radius()*CAMERA_SCALE;
        return particle_radius;
    }

    fn get_particle_num(&self) -> usize {
        return self.fluid_world.particles.positions.len();
    }

    fn reset_fluid(fluid_world: &mut sph::FluidParticleWorld3D) {
        fluid_world.remove_all_fluid_particles();
        fluid_world.remove_all_boundary_particles();

        fluid_world.add_fluid_rect(&Rect3D::new(0.1, 0.7, -0.1, 0.5, 1.0, 0.2), 0.05);

        /* fluid_world.add_boundary_thick_plane(Point3D::new(0.0, 0.0, -0.1), Point3D::new(2.0, 0.0, 0.1), 2);
        fluid_world.add_boundary_thick_plane(Point3D::new(0.0, 0.0, -0.1), Point3D::new(0.0, 2.5, 0.1), 2);
        fluid_world.add_boundary_thick_plane(Point3D::new(2.0, 0.0, -0.1), Point3D::new(2.0, 2.5, 0.1), 2);

        fluid_world.add_boundary_plane(Point3D::new(0.0, 0.6, -0.1), Point3D::new(1.75, 0.5, 0.1));
 */
        // close of the container - stop gap solution for issues with endlessly falling particles
        // (mostly a problem for adaptive timestep but potentially also for neighborhood search)
        // fluid_world.add_boundary_thick_plane(Point3D::new(0.0, 2.5, -0.1), Point::new(2.0, 2.5, 0.1), 2);
    }

    fn single_sim_step(&mut self) {
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

    fn reset_simulation(&mut self) {
        self.sph_solver.lock().unwrap().clear_cached_data(); // todo: this is super meh
        self.simulation_starttime = Instant::now();
        self.simulation_to_realtime_offset = 0.0;
        self.simulation_processing_time_total = Default::default();

        self.frame_counter = 0;
        self.time_manager.restart();
        Self::reset_fluid(&mut self.fluid_world);
    }
}
const GRAVITY: f32 = -9.821 * 100.0;
const SPRITE_SIZE: f32 = 3.0;

const SATURATION_DESELECTED: f32 = 0.3;
const LIGHTNESS_DESELECTED: f32 = 0.2;
const SATURATION_SELECTED: f32 = 0.9;
const LIGHTNESS_SELECTED: f32 = 0.7;
const ALPHA: f32 = 0.92;

const SHOWCASE_TIMER_SECS: f32 = 3.0;
const CAMERA_SCALE: f32 = 300.0;

fn main() {
    App::build()
        .add_plugins(DefaultPlugins)
        .add_plugin(LogDiagnosticsPlugin::default())
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .add_startup_system(setup.system())
        .add_system(move_system.system()) 
        .run();
}


fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {

    let mut camera = OrthographicCameraBundle::new_3d();
    camera.orthographic_projection.scale = 1.0;
    // camera.transform = Transform::from_xyz(5.0, 5.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y);
    camera.transform = Transform::from_xyz(5.0, 5.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y);

    // camera
    commands.spawn_bundle(camera);

    let particle_size = 0.03;
    let simState = MainState::new();
    let particle_radius = simState.get_particale_radius(); //*CAMERA_SCALE;

    println!("nbumber of particales: {}",simState.get_particle_num());
    println!("radius of particales: {}",particle_radius);
    
    for z in linspace::<f32>(-0.1, 0.1, 8) {

        for (p,vel) in simState.fluid_world.particles.positions.iter().zip(simState.fluid_world.particles.velocities.iter()) {
            let velocity = Vec3::new(vel[0], vel[1], vel[2]);
            // let pos = Point::new((p.x * CAMERA_SCALE) - 500.0, p.y * CAMERA_SCALE - 300.0);
            let pos = Point3D::new(p.x , p.y, p.z );
            // println!("position of particales: {}, {}, {}",pos.x, pos.y, pos.z);
            let transform = Transform::from_xyz(pos.x, pos.y, pos.z);
            commands
                .spawn()
                .insert_bundle((
                    Velocity {
                        translation: velocity,
                        rotation: 0.0
                    },
                ))
                .insert_bundle(PbrBundle {
                    mesh: meshes.add(Mesh::from(shape::Icosphere {radius: 0.01, subdivisions: 1})),
                    material: materials.add(Color::BLUE.into()),
                    transform: transform,
                    ..Default::default()
                })
                .id();
        }

    }
    /* for p in &simState.fluid_world.particles.boundary_particles {

        // let pos = Point::new((p.x * CAMERA_SCALE) - 500.0, p.y * CAMERA_SCALE -300.0);
        let pos = Point::new(p.x , p.y );
        let transform = Transform::from_xyz(pos.x, pos.y, 0.0);
        commands
            .spawn_bundle(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Icosphere {radius: 0.01, subdivisions: 1})),
                material: materials.add(Color::GRAY.into()),
                transform: transform,
                ..Default::default()
            })
            .id();
    }
 */
    commands.insert_resource(simState);
}

/// Apply velocity to positions and rotations.
fn move_system(time: Res<Time>, mut simState: ResMut<MainState>, mut q: Query<(&Velocity, &mut Transform)>) {
    let delta = time.delta_seconds();
    println!("steping...");
    simState.single_sim_step();
    
    let total_real_particles = simState.fluid_world.particles.positions.len();
    /* for (i, z) in linspace::<f32>(-0.1, 0.1, 8).enumerate() {
        for (j, (v, mut t)) in q.iter_mut()
            .enumerate()
            .filter(|(l,k)| l >= &(i*total_real_particles) && l < &((i+1) * total_real_particles)) {
             let p = &simState.fluid_world.particles.positions[j % total_real_particles];
             
             let transform = Vec3::new(p.x, p.y, z);
             t.translation = transform;
        }
    } */

    // }
}


