use std::collections::HashMap;

use crate::math::{point_line_distance::point_line_distance_unclamped, vector2::Vector2};

use super::rigidbody::RigidBody;

pub trait Constraint {
    fn solve(&self, bodies: &mut HashMap<String, RigidBody>, delta_timestep: f64) -> ();
}

pub struct DistanceJoint {
    pub distance: f64,
    pub strength: f64,
    pub body_a: String,
    pub body_b: String,
}

impl Constraint for DistanceJoint {
    fn solve(&self, bodies: &mut HashMap<String, RigidBody>, delta_timestep: f64) -> () {
        let a = bodies.get(&self.body_a).unwrap().clone();
        let b = bodies.get(&self.body_b).unwrap().clone();

        let delta = b.position - a.position;

        let offset = self.distance - delta.mag();

        let norm = delta.norm();

        let bias = (self.strength / delta_timestep) * offset;

        let relvel = b.velocity - a.velocity;

        let lambda = -(relvel.dot(&norm) - bias) / (a.inv_mass + b.inv_mass);

        {
            bodies.get_mut(&self.body_a).unwrap().velocity += norm * -lambda * a.inv_mass;
        }
        
        {
            bodies.get_mut(&self.body_b).unwrap().velocity += norm * lambda * b.inv_mass;
        }
    }
}

pub struct LookJoint {
    pub strength: f64,
    pub body_a: String,
    pub body_b: String,
}

impl Constraint for LookJoint {
    fn solve(&self, bodies: &mut HashMap<String, RigidBody>, delta_timestep: f64) -> () {
        let a = bodies.get(&self.body_a).unwrap().clone();
        let b = bodies.get(&self.body_b).unwrap().clone();

        let delta_pos = b.position - a.position;
        let norm = delta_pos.norm();

        let target_rot = f64::atan2(norm.y, norm.x);

        let ca = (target_rot - a.rotation).sin();
        let cb = (target_rot - b.rotation).sin();

        let bias_a = (self.strength / delta_timestep) * ca;
        let bias_b = (self.strength / delta_timestep) * cb;

        {
            let a = bodies.get_mut(&self.body_a).unwrap();
            a.ang_velocity = bias_a;
        }

        {
            let b = bodies.get_mut(&self.body_b).unwrap();
            b.ang_velocity = bias_b;
        }
    }
}

pub struct FixedJoint {
    pub body: String,
    pub position: Vector2,
    pub rotation: f64,
}

impl Constraint for FixedJoint {
    fn solve(&self, bodies: &mut HashMap<String, RigidBody>, _delta_timestep: f64) -> () {
        let body = bodies.get_mut(&self.body).unwrap();

        body.position = self.position;
        body.rotation = self.rotation;
        body.velocity = Vector2::zero();
        body.ang_velocity = 0.0;
    }
}

pub struct SlideJoint {
    pub body: String,
    pub position: Vector2,
    pub angle: f64,
    pub strength: f64,
}

impl Constraint for SlideJoint {
   fn solve(&self, bodies: &mut HashMap<String, RigidBody>, delta_timestep: f64) -> () {
        let body = bodies.get_mut(&self.body).unwrap();

        let dir = Vector2::new(self.angle.sin(), self.angle.cos());

        let mut closest_point = Vector2::zero();
        point_line_distance_unclamped(self.position, self.position + dir.perp(), body.position, Some(&mut closest_point));

        let delta = closest_point - body.position;

        let c = delta.mag();

        let bias = (self.strength / delta_timestep) * c;

        let impulse = -body.velocity.dot(&dir); 

        body.velocity += impulse * dir + bias * delta.norm();
   } 
}
