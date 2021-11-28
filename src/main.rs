use bevy::prelude::*;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy_prototype_lyon::prelude::*;

mod camera;

use bevysph::sph;
use bevysph::units;
use bevysph::units::{Point};

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

const GRAVITY: f32 = -9.821 * 100.0;
const SPRITE_SIZE: f32 = 3.0;

const SATURATION_DESELECTED: f32 = 0.3;
const LIGHTNESS_DESELECTED: f32 = 0.2;
const SATURATION_SELECTED: f32 = 0.9;
const LIGHTNESS_SELECTED: f32 = 0.7;
const ALPHA: f32 = 0.92;

const SHOWCASE_TIMER_SECS: f32 = 3.0;


fn main() {
    App::build()
        .insert_resource(Msaa {samples: 8})
        .add_plugins(DefaultPlugins)
        .add_plugin(LogDiagnosticsPlugin::default())
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .add_plugin(ShapePlugin)
        .add_startup_system(setup.system())
        .add_system(velocity_system.system())
        .add_system(move_system.system())
        .add_system(collision_system.system())
        .run();
}

fn reset_fluid(fluid_world: &mut sph::FluidParticleWorld) {
    fluid_world.remove_all_fluid_particles();
    fluid_world.remove_all_boundary_particles();

    fluid_world.add_fluid_rect(&units::Rect::new(0.1, 0.7, 0.5, 1.0), 0.05);
    fluid_world.add_boundary_thick_line(Point::new(0.0, 0.0), Point::new(2.0, 0.0), 2);
    fluid_world.add_boundary_thick_line(Point::new(0.0, 0.0), Point::new(0.0, 2.5), 2);
    fluid_world.add_boundary_thick_line(Point::new(2.0, 0.0), Point::new(2.0, 2.5), 2);

    fluid_world.add_boundary_line(Point::new(0.0, 0.6), Point::new(1.75, 0.5));

    // close of the container - stop gap solution for issues with endlessly falling particles
    // (mostly a problem for adaptive timestep but potentially also for neighborhood search)
    fluid_world.add_boundary_thick_line(Point::new(0.0, 2.5), Point::new(2.0, 2.5), 2);
}

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {

    /* commands.spawn_bundle(OrthographicCameraBundle::new_2d());
    commands.spawn_bundle(UiCameraBundle::default()); */
     // but with our custom projection

    let projection = SimpleOrthoProjection::default();

    // Need to set the camera name to one of the bevy-internal magic constants,
    // depending on which camera we are implementing (2D, 3D, or UI).
    // Bevy uses this name to find the camera and configure the rendering.
    // Since this example is a 2d camera:

    let cam_name = bevy::render::render_graph::base::camera::CAMERA_2D;

    let mut camera = Camera::default();
    camera.name = Some(cam_name.to_string());
    commands.spawn_bundle((
        // position the camera like bevy would do by default for 2D:
        Transform::from_translation(Vec3::new(0.0, 0.0, projection.far - 0.1)),
        GlobalTransform::default(),
        VisibleEntities::default(),
        camera,
        projection,
    ));


    let mut fluid_world = sph::FluidParticleWorld::new(
        2.0,    // smoothing factor
        1000.0, // #1660, 5000 particles/m²
        100.0,  // density of water (? this is 2d, not 3d where it's 1000 kg/m³)
    );
    reset_fluid(&mut fluid_world);
    let particle_radius = fluid_world.properties.particle_radius()*200.0;
        let shape = shapes::RegularPolygon {
                sides: 6,
                feature: shapes::RegularPolygonFeature::Radius(particle_radius),
                ..shapes::RegularPolygon::default()
            };

    println!("nbumber of particales: {}",fluid_world.particles.positions.len());
    println!("radius of particales: {}",particle_radius);

    for (pos,vel) in fluid_world.particles.positions.iter().zip(fluid_world.particles.velocities.iter()) {
        let velocity = Vec3::new(vel[0], vel[1], 0.0);

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

    for pos in &fluid_world.particles.boundary_particles {

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
    commands.insert_resource(fluid_world);
}

/// Finds the next contributor to display and selects the entity
fn add_particles(mut commands: Commands) {
    let particle_num = 100;
    for i in 1..particle_num{
        commands.spawn().insert(Particle)
            .insert(Name("Elaina Proctor".to_string()));
    }
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
    mut q: Query<(&mut Velocity, &mut Transform), With<sph::FluidParticleWorld>>,
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
fn move_system(time: Res<Time>, particle_world: ResMut<sph::FluidParticleWorld>, mut q: Query<(&Velocity, &mut Transform)>) {
    let delta = time.delta_seconds();
    // particle_world.timestep();
    

    for (v, mut t) in q.iter_mut() {
        t.translation += delta * v.translation;
        t.rotate(Quat::from_rotation_z(v.rotation * delta));
    }
}


