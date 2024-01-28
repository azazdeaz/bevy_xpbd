use bevy::prelude::*;

use crate::{InverseMass, RigidBodyQuery};

pub struct DragParticlePlugin;
impl Plugin for DragParticlePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, drag_particles);
    }
}

#[derive(Debug, Clone, Copy)]
struct DragInfo {
    id: Entity,
    grab_distance: f32,
    grab_inverse_mass: InverseMass,
}

#[derive(Component, Default)]
pub struct DragParticle {
    pub enabled: bool,
}
impl DragParticle {
    pub fn enabled() -> Self {
        Self { enabled: true }
    }
}

fn drag_particles(
    camera_query: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    mut particles: Query<(RigidBodyQuery, &DragParticle)>,
    buttons: Res<Input<MouseButton>>,
    windows: Query<&Window>,
    mut gizmos: Gizmos,
    mut drag_state: Local<Option<DragInfo>>,
) {
    let Some(cursor_position) = windows.single().cursor_position() else {
        warn!("No cursor position");
        return;
    };
    let Ok((camera, camera_transform)) = camera_query.get_single() else {
        warn!("No camera");
        return;
    };
    // Calculate a ray pointing from the camera into the world based on the cursor's position.
    let Some(ray) = camera.viewport_to_world(camera_transform, cursor_position) else {
        warn!("No ray");
        return;
    };

    if buttons.just_pressed(MouseButton::Left) {
        info!("Clicked");
        let mut closest: Option<(DragInfo, f32)> = None;
        for body in particles.into_iter().filter_map(
            |(body, drag)| {
                if drag.enabled {
                    Some(body)
                } else {
                    None
                }
            },
        ) {
            // calculate the particle distance from the ray
            let particle_from_origin = body.position.0 - ray.origin;
            let closest_point_on_ray = ray.direction * particle_from_origin.dot(ray.direction);
            let distance = (particle_from_origin - closest_point_on_ray).length();
            info!("Distance: {}", distance);
            if distance < 0.5 && (closest.is_none() || distance < closest.unwrap().1) {
                closest = Some((
                    DragInfo {
                        id: body.entity,
                        grab_distance: body.position.0.distance(ray.origin),
                        grab_inverse_mass: *body.inverse_mass,
                    },
                    distance,
                ));
            }
        }
        if let Some((info, _)) = closest.take() {
            *drag_state = Some(info);

            // Set inverse_mass to 0 to make the dragged body immovable
            if let Ok((mut body, _)) = particles.get_mut(info.id) {
                body.inverse_mass.0 = 0.0;
            }
        }
    } else if buttons.just_released(MouseButton::Left) {
        // Reset drag_state and inverse_mass
        if let Some(info) = drag_state.take() {
            if let Ok((mut body, _)) = particles.get_mut(info.id) {
                body.inverse_mass.0 = info.grab_inverse_mass.0;
            }
        }
    }

    if let Some(info) = &*drag_state {
        if let Ok((mut body, _)) = particles.get_mut(info.id) {
            info!("Dragging {:?}", info.id);
            let new_pos = ray.origin + ray.direction * info.grab_distance;
            body.position.0 = new_pos;
            gizmos.cuboid(
                Transform::from_translation(new_pos).with_scale(Vec3::splat(0.1)),
                Color::PINK,
            );
        }
    }
}
