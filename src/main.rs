use bevy::prelude::*;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
mod camera;

use bevysph::units::{Point, Real, Rect3D, Point3D};
use bevysph::main_state;
use std::{
    collections::BTreeSet,
};
use bevy::render::camera::{Camera, CameraProjection, DepthCalculation, VisibleEntities};
use itertools::linspace;

use isosurface::{
    distance::Signed,
    extractor::IndexedInterleavedNormals,
    feature::ParticleBasedMinimisation,
    implicit::{Cylinder, Difference, Intersection, RectangularPrism, Sphere, Torus, Union},
    // math::Vec3,
    sampler::Sampler,
    source::CentralDifference,
    DualContouring, ExtendedMarchingCubes, LinearHashedMarchingCubes, MarchingCubes,
};

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


struct Velocity {
    translation: Vec3,
    rotation: f32,
}

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
    // mut ambient_light: ResMut<AmbientLight>
) {

    let mut camera = OrthographicCameraBundle::new_3d();
    camera.orthographic_projection.scale = 2.5; //1.0;
    // camera.transform = Transform::from_xyz(5.0, 5.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y);
    camera.transform = Transform::from_xyz(0.0, 0.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y);
    // camera.transform = Transform::from_xyz(0.0, 5.0, 0.0).looking_at(Vec3::new(0.4, 0.0, 0.0), Vec3::Y);
    // camera.transform = Transform::from_xyz(0.0, 5.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y);

    // camera
    commands.spawn_bundle(camera);
    let particle_size = 0.03;
    let simState = main_state::MainState::new(
        2.0,
        1000.0,
        1000.0);
    let particle_radius = simState.get_particale_radius(); //*CAMERA_SCALE;
    let true_radius = simState.fluid_world.properties.particle_radius();
    println!("true particle_radius: {0}", true_radius);
    println!("nbumber of particales: {}",simState.get_particle_num());
    println!("radius of particales: {}",particle_radius);
    println!("smoothing_length: {}", simState.fluid_world.properties.smoothing_length());
    commands.spawn_bundle(LightBundle {
        // light: Light{color: Color::BLUE, ..Default::default()},
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..Default::default()
    }); 

    for (p,vel) in simState.fluid_world.particles.positions.iter().zip(simState.fluid_world.particles.velocities.iter()) {
        let velocity = Vec3::new(vel[0], vel[1], vel[2]);
        let pos = Point3D::new(p.x , p.y, p.z );
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
                mesh: meshes.add(Mesh::from(shape::Icosphere {radius: true_radius, subdivisions: 1})),
                material: materials.add(Color::BLUE.into()),
                transform: transform,
                ..Default::default()
            })
            .id();
    }

    let fluid_cm = simState.get_fluid_cm();
    let transform = Transform::from_xyz(fluid_cm.x, fluid_cm.y, fluid_cm.z);
    /* let mesh = simState.get_mesh(0.25);
    commands
        .spawn()
        .insert_bundle(PbrBundle {
            mesh: meshes.add(mesh),
            // material: materials.add(Color::WHITE.into()),
            material: materials.add(StandardMaterial{
                                        base_color: Color::Rgba{
                                            red: 0.35, 
                                            green: 0.35, 
                                            blue: 0.60, 
                                            alpha: 0.8}, 
                                        reflectance: 0.2, ..Default::default()}),
            transform: transform,
            ..Default::default()
        })
        .id(); */

    for p in &simState.fluid_world.particles.boundary_particles {

        // let pos = Point::new((p.x * CAMERA_SCALE) - 500.0, p.y * CAMERA_SCALE -300.0);
        let pos = Point3D::new(p.x , p.y, p.z );
        let transform = Transform::from_xyz(pos.x, pos.y, pos.z);
        commands
            .spawn_bundle(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Icosphere {radius: true_radius, subdivisions: 1})),
                material: materials.add(Color::WHITE.into()),
                transform: transform,
                ..Default::default()
            })
            .id();
    }

    commands.insert_resource(simState);
}

fn clamp(v: f32, min: f32, max: f32) -> f32 {
    if v < min {
        min
    } else if v > max {
        max
    } else {
        v
    }
}
fn heatmap_color(t: f32) -> Color {
    Color::Rgba {
        red: clamp(t * 3.0, 0.0, 1.0),
        green: clamp(t * 3.0 - 1.0, 0.0, 1.0),
        blue: clamp(t * 3.0 - 2.0, 0.0, 1.0),
        alpha: 1.0,
    }
}

/// Apply velocity to positions and rotations.
fn move_system(time: Res<Time>, mut simState: ResMut<main_state::MainState>,
               mut q: Query<(&Velocity, &mut Transform, &Handle<StandardMaterial>, &Handle<Mesh>)>, 
               mut materials: ResMut<Assets<StandardMaterial>>,
               mut meshes: ResMut<Assets<Mesh>> ) {
    let delta = time.delta_seconds();
    simState.single_sim_step();
    let cm = simState.get_fluid_cm();
    // for (j, (_, mut t, mat_handle, mesh_handle)) in q.iter_mut()
        // .enumerate(){
         // let transform = Vec3::new(cm.x, cm.y, cm.z);
         // t.translation = transform;
         /* let mesh = &mut meshes.get_mut(mesh_handle).unwrap();
         let lbox = simState.get_fluid_bbox();
         let newmesh = simState.get_vertices(lbox);
         mesh.set_attribute(Mesh::ATTRIBUTE_POSITION, newmesh); */
    // }
    for (j, (_, mut t, mat_handle, mesh_handle)) in q.iter_mut()
        .enumerate(){
         let p = &simState.fluid_world.particles.positions[j];
         let v = &simState.fluid_world.particles.velocities[j];
         let transform = Vec3::new(p.x, p.y, p.z);
         t.translation = transform;
         /* let color = &mut materials.get_mut(handle).unwrap();
         color.base_color = heatmap_color((v.x*v.x+v.y*v.y+v.z*v.z).sqrt()); */
    }

    // }
}


