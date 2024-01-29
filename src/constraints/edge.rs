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

/// A compliance of 0.0 resembles a constraint with infinite stiffness, so the bodies should not have any overlap.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(MapEntities)]
pub struct EdgeConstraint {
    /// First entity in the constraint.
    pub entity1: Entity,
    /// Second entity in the constraint.
    pub entity2: Entity,
    /// Resing volume
    pub rest_length: Scalar,
    /// The constraint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
}
impl XpbdConstraint<2> for EdgeConstraint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }

    fn clear_lagrange_multipliers(&mut self) {
        // self.normal_lagrange = 0.0;
        // self.tangent_lagrange = 0.0;
    }

    /// Solves overlap between two bodies.
    fn solve(&mut self, bodies: [&mut RigidBodyQueryItem; 2], dt: Scalar) {
        let inv_mass1 = bodies[0].inverse_mass.0;
        let inv_mass2 = bodies[1].inverse_mass.0;
        let w = inv_mass1 + inv_mass2;
        if w == 0.0 {
            return;
        }
        let alpha = self.compliance / (dt * dt);
        let p1 = bodies[0].current_position();
        let p2 = bodies[1].current_position();

        let delta = p2 - p1;
        let distance = delta.length();
        let direction = if distance == 0.0 {
            // Choose a random direction if the edge is collapsed.
            warn!("Edge constraint has zero length. Choosing random direction to separate the particles.");
            Vec3::X
        } else {
            delta / distance
        };
        let residual = -(distance - self.rest_length) / (w + alpha);
        // println!("edge {}->{} residual: {}", self.entity1.index(), self.entity2.index(), residual);
        bodies[0].accumulated_translation.0 -= direction * residual * inv_mass1;
        bodies[1].accumulated_translation.0 += direction * residual * inv_mass2;
    }
}

pub fn draw_debug_edge_constraints(
    mut gizmos: Gizmos,
    constraints: Query<&EdgeConstraint>,
    transforms: Query<&GlobalTransform>,
) {
    for constraint in constraints.iter() {
        let Some((p1, p2)) = [constraint.entity1, constraint.entity2]
            .iter()
            .filter_map(|entity| transforms.get(*entity).map(|t| t.translation()).ok())
            .collect_tuple()
        else {
            return;
        };

        gizmos.line(p1, p2, Color::hex("#D95B66").unwrap());
    }
}

impl EdgeConstraint {
    /// Creates a new [`EdgeConstraint`] with the given bodies and contact data.
    pub fn new(entity1: &Entity, position1: &Vec3, entity2: &Entity, position2: &Vec3) -> Self {
        let rest_length = position1.distance(*position2);

        Self {
            entity1: *entity1,
            entity2: *entity2,
            rest_length,
            compliance: 0.1,
        }
    }

    /// Sets the constrains's compliance (inverse of stiffness, meters / Newton).
    pub fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.compliance = compliance;
        self
    }

    pub fn with_rest_length(mut self, rest_length: Scalar) -> Self {
        self.rest_length = rest_length;
        self
    }
}

impl MapEntities for EdgeConstraint {
    fn map_entities(&mut self, entity_mapper: &mut EntityMapper) {
        self.entity1 = entity_mapper.get_or_reserve(self.entity1);
        self.entity2 = entity_mapper.get_or_reserve(self.entity2);
    }
}
