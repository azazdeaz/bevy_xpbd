use bevy::prelude::*;

pub struct DragParticlePlugin;
impl Plugin for DragParticlePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, drag_particles);
    }
}

struct DragInfo {
    id: Entity,
    grab_distance: f32,
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
    mut particles: Query<(Entity, &mut Transform, &DragParticle)>,
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
        let mut closest: Option<(Entity, f32)> = None;
        for (id, particle) in particles.into_iter().filter_map(|(id, transform, drag)| {
            if drag.enabled {
                Some((id, transform))
            } else {
                None
            }
        }) {
            // calculate the particle distance from the ray
            let particle_from_origin = particle.translation - ray.origin;
            let closest_point_on_ray = ray.direction * particle_from_origin.dot(ray.direction);
            let distance = (particle_from_origin - closest_point_on_ray).length();
            info!("Distance: {}", distance);
            if distance < 0.5 && (closest.is_none() || distance < closest.unwrap().1) {
                closest = Some((id, distance));
            }
        }
        if let Some((id, _)) = closest {
            *drag_state = Some(DragInfo {
                id,
                grab_distance: particles
                    .get(id)
                    .unwrap()
                    .1
                    .translation
                    .distance(ray.origin),
            });
        }
    } else if buttons.just_released(MouseButton::Left) {
        *drag_state = None;
    }

    if let Some(info) = &*drag_state {
        info!("Dragging {:?}", info.id);
        let new_pos = ray.origin + ray.direction * info.grab_distance;
        let mut particle_position = particles.get_mut(info.id).unwrap().1;
        particle_position.translation = new_pos;
        gizmos.cuboid(
            Transform::from_translation(new_pos).with_scale(Vec3::splat(0.1)),
            Color::PINK,
        );
    }
}
