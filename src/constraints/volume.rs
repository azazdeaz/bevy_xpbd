//! Penetration constraint.

use crate::prelude::*;
use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};
use itertools::Itertools;

/// A constraint between two bodies that prevents overlap with a given compliance.
///
/// A compliance of 0.0 resembles a constraint with infinite stiffness, so the bodies should not have any overlap.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(MapEntities)]
pub struct VolumeConstraint {
    /// First entity in the constraint.
    pub entity1: Entity,
    /// Second entity in the constraint.
    pub entity2: Entity,
    /// Third entity in the constraint.
    pub entity3: Entity,
    /// Fourth entity in the constraint.
    pub entity4: Entity,
    /// Resing volume
    pub rest_volume: Scalar,
    /// The constraint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
}
impl XpbdConstraint<4> for VolumeConstraint {
    fn entities(&self) -> [Entity; 4] {
        [self.entity1, self.entity2, self.entity3, self.entity4]
    }

    fn clear_lagrange_multipliers(&mut self) {
        // self.normal_lagrange = 0.0;
        // self.tangent_lagrange = 0.0;
    }

    /// Change the particle's position to satisfy the constraint.
    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 4], dt: Scalar) {
        let alpha = self.compliance / (dt * dt);
        let p1 = bodies[0].current_position();
        let p2 = bodies[1].current_position();
        let p3 = bodies[2].current_position();
        let p4 = bodies[3].current_position();
        let mut w = 0.0;
        // all combinations of [body.inverse_mass, ...positions of opposite bodies]
        // TODO: I think the order doesnt matter because ||AB x AC||^2 is the same regardless of the order, but i should check if this is true
        let id_views = [
            (bodies[0].inverse_mass.0, p2, p3, p4),
            (bodies[1].inverse_mass.0, p1, p4, p3),
            (bodies[3].inverse_mass.0, p1, p3, p2),
            (bodies[2].inverse_mass.0, p1, p2, p4),
        ];
        let gradients = id_views
            .iter()
            .map(|(inverse_mass, pa, pb, pc)| {
                let gradient = (*pb - *pa).cross(*pc - *pa) / 6.0;
                w += inverse_mass * gradient.length_squared();
                gradient
            })
            .collect_vec();
        if w == 0.0 {
            return;
        }
        let volume = Self::volume(&p1, &p2, &p3, &p4);
        // println!("p1 {}, p2 {}, p3 {}, p4 {}", p1, p2, p3, p4);
        let residual = -(volume - self.rest_volume) / (w + alpha);
        // println!(
        //     "volume {}, rest_volume, {}, residual {}",
        //     volume, self.rest_volume, residual
        // );
        for (index, gradient) in gradients.into_iter().enumerate() {
            let inverse_mass = bodies[index].inverse_mass.0;
            let push = gradient * residual * inverse_mass;
            // println!("push {}", push);
            if push.is_nan() {
                panic!("push is nan");
            }
            bodies[index].accumulated_translation.0 += push;
        }
    }
}

pub fn draw_debug_volume_constraints(
    mut gizmos: Gizmos,
    constraints: Query<&VolumeConstraint>,
    transforms: Query<&GlobalTransform>,
) {
    for constraint in constraints.iter() {
        let Some((p1, p2, p3, p4)) = [
            constraint.entity1,
            constraint.entity2,
            constraint.entity3,
            constraint.entity4,
        ]
        .iter()
        .filter_map(|entity| transforms.get(*entity).map(|t| t.translation()).ok())
        .collect_tuple() else {
            return;
        };
        let center = (p1 + p2 + p3 + p4) / 4.0;
        // Shrink the tetrahedron to make it easier to see
        let scale = 0.9;
        let p1 = center + (p1 - center) * scale;
        let p2 = center + (p2 - center) * scale;
        let p3 = center + (p3 - center) * scale;
        let p4 = center + (p4 - center) * scale;

        let color = Color::hex("#F2A2E5").unwrap();
        gizmos.line(p1, p2, color);
        gizmos.line(p1, p3, color);
        gizmos.line(p1, p4, color);
        gizmos.line(p2, p3, color);
        gizmos.line(p2, p4, color);
        gizmos.line(p3, p4, color);
    }
}

impl VolumeConstraint {
    /// Creates a new [`VolumeConstraint`] with the given bodies and contact data.
    pub fn new(
        entity1: &Entity,
        position1: &Vec3,
        entity2: &Entity,
        position2: &Vec3,
        entity3: &Entity,
        position3: &Vec3,
        entity4: &Entity,
        position4: &Vec3,
    ) -> Self {
        let mut rest_volume = Self::volume(&position1, &position2, &position3, &position4);
        // Swap entity2 and entity3 if the volume is negative
        let (entity2, entity3) = if rest_volume < 0.0 {
            rest_volume = Self::volume(&position1, &position3, &position2, &position4);
            (entity3, entity2)
        } else {
            (entity2, entity3)
        };
        assert!(
            rest_volume > 0.0,
            "initial rest_volume is still not positive (={}) after swapping entity2 and entity3",
            rest_volume
        );
        Self {
            entity1: *entity1,
            entity2: *entity2,
            entity3: *entity3,
            entity4: *entity4,
            rest_volume,
            compliance: 0.0,
        }
    }

    pub fn volume(p1: &Vector, p2: &Vector, p3: &Vector, p4: &Vector) -> f32 {
        let v1 = *p2 - *p1;
        let v2 = *p3 - *p1;
        let v3 = *p4 - *p1;
        v1.cross(v2).dot(v3) / 6.0
    }

    /// Sets the constrains's compliance (inverse of stiffness, meters / Newton).
    pub fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.compliance = compliance;
        self
    }

    pub fn with_rest_volume(mut self, rest_volume: Scalar) -> Self {
        self.rest_volume = rest_volume;
        self
    }
}

impl MapEntities for VolumeConstraint {
    fn map_entities(&mut self, entity_mapper: &mut EntityMapper) {
        self.entity1 = entity_mapper.get_or_reserve(self.entity1);
        self.entity2 = entity_mapper.get_or_reserve(self.entity2);
        self.entity3 = entity_mapper.get_or_reserve(self.entity3);
        self.entity4 = entity_mapper.get_or_reserve(self.entity4);
    }
}
