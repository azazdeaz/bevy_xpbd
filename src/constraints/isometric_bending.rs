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

///              pb
///            /-^<\
///       e1/--  |  --\e4
///      /--     |     --\
/// pd <-        e0       -> pc
///      \--     |     --/
///       e2\--  |  --/e3
///            \>|-/
///              pa
///
/// A compliance of 0.0 resembles a constraint with infinite stiffness, so the bodies should not have any overlap.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[reflect(MapEntities)]
pub struct IsometricBendingConstraint {
    /// Pivot particle (shared by both triangles).
    pub entity1: Entity,
    /// The other particle shared by both triangles.
    pub entity2: Entity,
    /// Head of the first triangle.
    pub entity3: Entity,
    /// Head of the second triangle.
    pub entity4: Entity,
    // Q
    pub initial_bending_energy: Mat4,
    /// The constraint's compliance, the inverse of stiffness, has the unit meters / Newton.
    pub compliance: Scalar,
}
impl XpbdConstraint<4> for IsometricBendingConstraint {
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
        let pa = bodies[0].current_position();
        let pb = bodies[1].current_position();
        let pc = bodies[2].current_position();
        let pd = bodies[3].current_position();

        let ima = bodies[0].inverse_mass.0;
        let imb = bodies[1].inverse_mass.0;
        let imc = bodies[2].inverse_mass.0;
        let imd = bodies[3].inverse_mass.0;

        let energy = self.calculate_constraint_value(&pa, &pb, &pc, &pd);
        if energy < 1e-12 {
            return;
        }

        let grad = self.calculate_gradient(&pa, &pb, &pc, &pd);
        let mut sum_normal_grad = 0.0;
        sum_normal_grad += ima * grad[0].length_squared();
        sum_normal_grad += imb * grad[1].length_squared();
        sum_normal_grad += imc * grad[2].length_squared();
        sum_normal_grad += imd * grad[3].length_squared();

        if sum_normal_grad.abs() < 1e-9 {
            return;
        }

        // compute impulse-based scaling factor
        let s = energy / sum_normal_grad;

        bodies[0].accumulated_translation.0 += s * ima * grad[0] * alpha;
        bodies[1].accumulated_translation.0 += s * imb * grad[1] * alpha;
        bodies[2].accumulated_translation.0 += s * imc * grad[2] * alpha;
        bodies[3].accumulated_translation.0 += s * imd * grad[3] * alpha;
    }
}

pub fn draw_debug_isometric_bend_constraints(
    mut gizmos: Gizmos,
    constraints: Query<&IsometricBendingConstraint>,
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

        gizmos.line(p1, p2, Color::PURPLE);
        gizmos.line(p1, p3, Color::PURPLE);
        gizmos.line(p1, p4, Color::PURPLE);
        gizmos.line(p2, p3, Color::PURPLE);
        gizmos.line(p2, p4, Color::PURPLE);
        gizmos.line(p3, p4, Color::PURPLE);
    }
}

impl IsometricBendingConstraint {
    /// Creates a new [`IsometricBendingConstraint`] with the given bodies and contact data.
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
        Self {
            entity1: *entity1,
            entity2: *entity2,
            entity3: *entity3,
            entity4: *entity4,
            initial_bending_energy: Self::get_bending_energy(
                position1, position2, position3, position4,
            ),
            compliance: 0.0,
        }
    }

    // Q
    fn get_bending_energy(pa: &Vector, pb: &Vector, pc: &Vector, pd: &Vector) -> Mat4 {
        // Calculate edge lengths
        let e0 = *pb - *pa;
        let e1 = *pd - *pb;
        let e2 = *pa - *pd;
        let e3 = *pc - *pa;
        let e4 = *pb - *pc;

        let area_left = e0.cross(*pd - *pa).length() * 0.5;
        let area_right = e0.cross(e3).length() * 0.5;
        let area = area_left + area_right;

        let cot = |v0: Vec3, v1: Vec3| -> f32 { v0.dot(v1) / v0.cross(v1).length() };
        // K = (c01 + c04, c02 + c03, −c01 − c02, −c03 − c04)
        let c01 = cot(e0, e1);
        let c02 = cot(e0, e2);
        let c03 = cot(e0, e3);
        let c04 = cot(e0, e4);
        let k = Vec4::new(c01 + c04, c02 + c03, -c01 - c02, -c03 - c04);
        outer_product(k, k) * (3.0 / area)
    }

    fn calculate_constraint_value(
        &self,
        pa: &Vector,
        pb: &Vector,
        pc: &Vector,
        pd: &Vector,
    ) -> f32 {
        let mut sum = 0.0;
        let particles = [pa, pb, pc, pd];
        for i in 0..4 {
            for j in 0..4 {
                sum += self.initial_bending_energy.row(i)[j] * particles[i].dot(*particles[j]);
            }
        }
        sum / 2.0
    }

    fn calculate_gradient(&self, pa: &Vector, pb: &Vector, pc: &Vector, pd: &Vector) -> Vec<Vec3> {
        let mut gradient = vec![Vec3::default(); 4];
        let particles = [pa, pb, pc, pd];
        for i in 0..4 {
            for j in 0..4 {
                gradient[i] += self.initial_bending_energy.row(i)[j] * *particles[j];
            }
        }
        gradient
    }

    /// Sets the constrains's compliance (inverse of stiffness, meters / Newton).
    pub fn with_compliance(mut self, compliance: Scalar) -> Self {
        self.compliance = compliance;
        self
    }
}

fn outer_product(v0: Vec4, v1: Vec4) -> Mat4 {
    Mat4::from_cols(
        Vec4::new(v0.x * v1.x, v0.x * v1.y, v0.x * v1.z, v0.x * v1.w),
        Vec4::new(v0.y * v1.x, v0.y * v1.y, v0.y * v1.z, v0.y * v1.w),
        Vec4::new(v0.z * v1.x, v0.z * v1.y, v0.z * v1.z, v0.z * v1.w),
        Vec4::new(v0.w * v1.x, v0.w * v1.y, v0.w * v1.z, v0.w * v1.w),
    )
}

impl MapEntities for IsometricBendingConstraint {
    fn map_entities(&mut self, entity_mapper: &mut EntityMapper) {
        self.entity1 = entity_mapper.get_or_reserve(self.entity1);
        self.entity2 = entity_mapper.get_or_reserve(self.entity2);
        self.entity3 = entity_mapper.get_or_reserve(self.entity3);
        self.entity4 = entity_mapper.get_or_reserve(self.entity4);
    }
}
