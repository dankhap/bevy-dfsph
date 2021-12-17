use bevy::prelude::*;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy_prototype_lyon::prelude::*;
// use cgmath::prelude::*;
use std::collections::VecDeque;
use std::time::{Duration, Instant};
mod camera;

use std::sync::{Arc, Mutex};
use bevysph::sph;
use bevysph::units::{Point, Real, Rect};

use rand::{prelude::SliceRandom, Rng};
use std::{
    collections::BTreeSet,
};
use bevy::render::camera::{Camera, CameraProjection, DepthCalculation, VisibleEntities};

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

pub struct HelloPlugin;
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
    fluid_world: sph::FluidParticleWorld,
    time_manager: sph::TimeManager,
    sph_solver: Arc<Mutex<dyn sph::Solver + Send>>,

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

        let mut fluid_world = sph::FluidParticleWorld::new(
            2.0,    // smoothing factor
            1000.0, // #1660, 5000 particles/m²
            100.0,  // density of water (? this is 2d, not 3d where it's 1000 kg/m³)
        );
        let xsph = sph::XSPHViscosityModel::new(fluid_world.properties.smoothing_length());
        //xsph.epsilon = 0.1;
        let mut physicalviscosity = 
            sph::PhysicalViscosityModel::new(fluid_world.properties.smoothing_length());
        physicalviscosity.fluid_viscosity = 0.01;

        let sph_solver: Arc<Mutex<dyn sph::Solver + Send> > = 
            Arc::new(Mutex::new(sph::DFSPHSolver::new(xsph, fluid_world.properties.smoothing_length())));


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

    fn reset_fluid(fluid_world: &mut sph::FluidParticleWorld) {
        fluid_world.remove_all_fluid_particles();
        fluid_world.remove_all_boundary_particles();

        fluid_world.add_fluid_rect(&Rect::new(0.1, 0.7, 0.5, 1.0), 0.05);
        fluid_world.add_boundary_thick_line(Point::new(0.0, 0.0), Point::new(2.0, 0.0), 2);
        fluid_world.add_boundary_thick_line(Point::new(0.0, 0.0), Point::new(0.0, 2.5), 2);
        fluid_world.add_boundary_thick_line(Point::new(2.0, 0.0), Point::new(2.0, 2.5), 2);

        fluid_world.add_boundary_line(Point::new(0.0, 0.6), Point::new(1.75, 0.5));

        // close of the container - stop gap solution for issues with endlessly falling particles
        // (mostly a problem for adaptive timestep but potentially also for neighborhood search)
        fluid_world.add_boundary_thick_line(Point::new(0.0, 2.5), Point::new(2.0, 2.5), 2);
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
        .insert_resource(Msaa {samples: 8})
        .add_plugins(DefaultPlugins)
        .add_plugin(LogDiagnosticsPlugin::default())
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .add_plugin(ShapePlugin)
        .add_startup_system(setup.system())
            /* .add_system(velocity_system.system()) */
        .add_system(move_system.system()) 
        // .add_system(collision_system.system())
        .run();
}


fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {

    commands.spawn_bundle(OrthographicCameraBundle::new_2d());
    commands.spawn_bundle(UiCameraBundle::default());
     // but with our custom projection

    // let projection = SimpleOrthoProjection::default();

    // Need to set the camera name to one of the bevy-internal magic constants,
    // depending on which camera we are implementing (2D, 3D, or UI).
    // Bevy uses this name to find the camera and configure the rendering.
    // Since this example is a 2d camera:

    // let cam_name = bevy::render::render_graph::base::camera::CAMERA_2D;

    /* let mut camera = Camera::default();
    camera.name = Some(cam_name.to_string());
    commands.spawn_bundle((
        // position the camera like bevy would do by default for 2D:
        Transform::from_translation(Vec3::new(0.0, 0.0, projection.far - 0.1)),
        GlobalTransform::default(),
        VisibleEntities::default(),
        camera,
        projection,
    )); */


    let simState = MainState::new();
    let particle_radius = simState.get_particale_radius(); //*CAMERA_SCALE;

        let shape = shapes::RegularPolygon {
                sides: 6,
                feature: shapes::RegularPolygonFeature::Radius(particle_radius),
                ..shapes::RegularPolygon::default()
            };

    println!("nbumber of particales: {}",simState.get_particle_num());
    println!("radius of particales: {}",particle_radius);

    for (p,vel) in simState.fluid_world.particles.positions.iter().zip(simState.fluid_world.particles.velocities.iter()) {
        let velocity = Vec3::new(vel[0], vel[1], 0.0);
        let pos = Point::new((p.x * CAMERA_SCALE) - 500.0, p.y * CAMERA_SCALE - 300.0);
        println!("position of particales: {}, {}",pos.x, pos.y);
        let transform = Transform::from_xyz(pos.x, pos.y, 0.0);
        let e = commands
            .spawn()
            .insert_bundle((
                Velocity {
                    translation: velocity,
                    rotation: 0.0
                },
            ))
            .insert_bundle(GeometryBuilder::build_as(
                &shape,
                ShapeColors::outlined(Color::BLUE, Color::BLACK),
                DrawMode::Outlined {
                    fill_options: FillOptions::default(),
                    outline_options: StrokeOptions::default().with_line_width(1.0),
                },
                transform,
            ))
            .id();
    }

    for p in &simState.fluid_world.particles.boundary_particles {

        let pos = Point::new((p.x * CAMERA_SCALE) - 500.0, p.y * CAMERA_SCALE -300.0);
        // let pos = Point::new((p.x * 800.0) - 400.0, p.y * 800.0);
        let transform = Transform::from_xyz(pos.x, pos.y, 0.0);
        let shape = shapes::RegularPolygon {
                sides: 6,
                feature: shapes::RegularPolygonFeature::Radius(particle_radius),
                ..shapes::RegularPolygon::default()
            };
        commands
            .spawn()
            .insert_bundle(GeometryBuilder::build_as(
                &shape,
                ShapeColors::outlined(Color::GRAY, Color::BLACK),
                DrawMode::Outlined {
                    fill_options: FillOptions::default(),
                    outline_options: StrokeOptions::default().with_line_width(1.0),
                },
                transform,
            ))
            .id();
    }
    commands.insert_resource(simState);
    // commands.insert_resource(fluid_world);
    // commands.insert_resource(sph_solver);
}

/// Applies gravity to all entities with velocity
fn velocity_system(time: Res<Time>, mut q: Query<&mut Velocity>) {
    let delta = time.delta_seconds();

    for mut v in q.iter_mut() {
        v.translation += Vec3::new(0.0, GRAVITY * delta, 0.0);
    }
}

/// Checks for collisions of contributor-birds.
///
/// On collision with left-or-right wall it resets the horizontal
/// velocity. On collision with the ground it applies an upwards
/// force.
fn collision_system(
    wins: Res<Windows>,
    mut q: Query<(&mut Velocity, &mut Transform), With<MainState>>,
) {
    let mut rnd = rand::thread_rng();

    let win = wins.get_primary().unwrap();

    let ceiling = win.height() / 2.;
    let ground = -(win.height() / 2.);

    let wall_left = -(win.width() / 2.);
    let wall_right = win.width() / 2.;

    for (mut v, mut t) in q.iter_mut() {
        let left = t.translation.x - SPRITE_SIZE / 2.0;
        let right = t.translation.x + SPRITE_SIZE / 2.0;
        let top = t.translation.y + SPRITE_SIZE / 2.0;
        let bottom = t.translation.y - SPRITE_SIZE / 2.0;

        // clamp the translation to not go out of the bounds
        if bottom < ground {
            t.translation.y = ground + SPRITE_SIZE / 2.0;
            // apply an impulse upwards
            v.translation.y = rnd.gen_range(700.0..1000.0);
        }
        if top > ceiling {
            t.translation.y = ceiling - SPRITE_SIZE / 2.0;
        }
        // on side walls flip the horizontal velocity
        if left < wall_left {
            t.translation.x = wall_left + SPRITE_SIZE / 2.0;
            v.translation.x *= -1.0;
            v.rotation *= -1.0;
        }
        if right > wall_right {
            t.translation.x = wall_right - SPRITE_SIZE / 2.0;
            v.translation.x *= -1.0;
            v.rotation *= -1.0;
        }
    }
}

/// Apply velocity to positions and rotations.
fn move_system(time: Res<Time>, mut simState: ResMut<MainState>, mut q: Query<(&Velocity, &mut Transform)>) {
    let delta = time.delta_seconds();
    simState.single_sim_step();
    

    for (i, (v, mut t)) in q.iter_mut().enumerate() {
        let allp = &simState.fluid_world.particles.positions;
        if allp.len() - 1 < i {
            return
        }
        let p = allp[i];
        let pos = Point::new((p.x * CAMERA_SCALE) - 500.0, p.y * CAMERA_SCALE - 300.0);
        // println!("position of particales: {}, {}",pos.x, pos.y);
        let transform = Vec3::new(pos.x, pos.y, 0.0);
        t.translation = transform; 
        // t.translation += delta * v.translation;
    }
}


