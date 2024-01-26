//! This example is a version of Bevy's `3d_shapes` example that uses trimesh colliders for the shapes.
//!
//! You could also use convex decomposition to generate compound shapes from Bevy meshes.
//! The decomposition algorithm can be slow, but the generated colliders are often faster and more robust
//! than trimesh colliders.

use std::f32::consts::FRAC_PI_2;

use bevy::{
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
};
use bevy_xpbd_3d::prelude::*;
use examples_common_3d::XpbdExamplePlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(ImagePlugin::default_nearest()),
            XpbdExamplePlugin,
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let debug_material = materials.add(StandardMaterial {
        base_color_texture: Some(images.add(uv_debug_texture())),
        ..default()
    });

    let body_width = 1.0;
    let body_length = 2.0;
    let start_height = 3.0;
    let wheel_positions = vec![
        Vec3::new(body_width / 2.0, 0.0, body_length / 2.0),
        // Vec3::new(-body_width / 2.0, 0.0, body_length / 2.0),
        // Vec3::new(body_width / 2.0, 0.0, -body_length / 2.0),
        // Vec3::new(-body_width / 2.0, 0.0, -body_length / 2.0),
    ];
    let body_mesh = shape::Box::new(body_width / 10.0, 0.5, body_length / 10.0).into();
    let wheel_mesh = shape::Cylinder {
        radius: 0.4,
        height: 0.2,
        ..Default::default()
    }
    .into();

    let body = commands
        .spawn((
            RigidBody::Dynamic,
            Collider::trimesh_from_mesh(&body_mesh).unwrap(),
            PbrBundle {
                mesh: meshes.add(body_mesh),
                material: debug_material.clone(),
                transform: Transform::from_xyz(0.0, start_height, 0.0),
                ..default()
            },
        ))
        .id();

    for pos in wheel_positions {
        let wheel = commands.spawn((
            RigidBody::Dynamic,
            Collider::trimesh_from_mesh(&wheel_mesh).unwrap(),
            PbrBundle {
                mesh: meshes.add(wheel_mesh.clone()),
                material: debug_material.clone(),
                transform: Transform::from_translation(pos + Vec3::Y * start_height)
                ,
                // .with_rotation(Quat::from_rotation_z(FRAC_PI_2)),
                ..default()
            },
        )).id();

        commands.spawn(
            FixedJoint::new(body, wheel)
                .with_local_anchor_2(-pos)
                // .with_angle_limits(-1.0, 1.0),
        );
    }

    // Ground plane
    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(50.0, 0.1, 50.0),
        PbrBundle {
            mesh: meshes.add(shape::Plane::from_size(50.0).into()),
            material: materials.add(Color::SILVER.into()),
            transform: Transform::from_xyz(0.0, -1.0, 0.0),
            ..default()
        },
    ));

    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 9000.0,
            range: 100.,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(8.0, 16.0, 8.0),
        ..default()
    });

    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 6., 12.0).looking_at(Vec3::new(0., 1., 0.), Vec3::Y),
        ..default()
    });
}

/// Creates a colorful test pattern
fn uv_debug_texture() -> Image {
    const TEXTURE_SIZE: usize = 8;

    let mut palette: [u8; 32] = [
        255, 102, 159, 255, 255, 159, 102, 255, 236, 255, 102, 255, 121, 255, 102, 255, 102, 255,
        198, 255, 102, 198, 255, 255, 121, 102, 255, 255, 236, 102, 255, 255,
    ];

    let mut texture_data = [0; TEXTURE_SIZE * TEXTURE_SIZE * 4];
    for y in 0..TEXTURE_SIZE {
        let offset = TEXTURE_SIZE * y * 4;
        texture_data[offset..(offset + TEXTURE_SIZE * 4)].copy_from_slice(&palette);
        palette.rotate_right(4);
    }

    Image::new_fill(
        Extent3d {
            width: TEXTURE_SIZE as u32,
            height: TEXTURE_SIZE as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &texture_data,
        TextureFormat::Rgba8UnormSrgb,
    )
}
