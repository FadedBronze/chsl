use std::collections::HashMap;

use crate::math::{point_line_distance::point_line_distance_unclamped, vector2::Vector2};

use super::rigidbody::RigidBody;

pub enum Constraint {
    DistanceJoint {
        distance: f64,
        strength: f64,
        body_a: String,
        body_b: String,
    },
    LookJoint {
        strength: f64,
        body_a: String,
        body_b: String,
    },
    FixedJoint {
        body: String,
        position: Vector2,
        rotation: f64,
        strength: f64
    },
    SlideJoint {
        body: String,
        position: Vector2,
        rotation: f64,
        strength: f64,
    }
}

impl Constraint {
    pub fn solve(&self, bodies: &mut HashMap<String, RigidBody>, delta_timestep: f64) -> () {
        match self {
            Self::DistanceJoint { distance, strength, body_a, body_b } => {
                let a = bodies.get(body_a).unwrap().clone();
                let b = bodies.get(body_b).unwrap().clone();

                let delta = b.position - a.position;

                let offset = distance - delta.mag();

                let norm = delta.norm();

                let bias = (strength / delta_timestep) * offset;

                let relvel = b.velocity - a.velocity;

                let lambda = -(relvel.dot(&norm) - bias) / (a.inv_mass + b.inv_mass);

                {
                    bodies.get_mut(body_a).unwrap().velocity += norm * -lambda * a.inv_mass;
                }
                
                {
                    bodies.get_mut(body_b).unwrap().velocity += norm * lambda * b.inv_mass;
                }
            }
            Self::LookJoint { strength, body_a, body_b } => {
                let a = bodies.get(body_a).unwrap().clone();
                let b = bodies.get(body_b).unwrap().clone();

                let delta_pos = b.position - a.position;
                let norm = delta_pos.norm();

                let target_rot = f64::atan2(norm.y, norm.x);

                let ca = (target_rot - a.rotation).sin();
                let cb = (target_rot - b.rotation).sin();

                let bias_a = (strength / delta_timestep) * ca;
                let bias_b = (strength / delta_timestep) * cb;

                {
                    let a = bodies.get_mut(body_a).unwrap();
                    a.ang_velocity = bias_a;
                }

                {
                    let b = bodies.get_mut(body_b).unwrap();
                    b.ang_velocity = bias_b;
                }
            }
            Self::FixedJoint { body, position, rotation, strength } => {
                let body = bodies.get_mut(body).unwrap();

                let c_pos = *position - body.position;
                let bias_pos = (strength / delta_timestep) * c_pos;

                let c_rot = (*rotation - body.rotation).sin();
                let bias_rot = (strength / delta_timestep) * c_rot;

                body.velocity = bias_pos;
                body.ang_velocity = bias_rot;
            }
            Self::SlideJoint { body, position, rotation: angle, strength } => {
                let body = bodies.get_mut(body).unwrap();

                let dir = Vector2::new(angle.sin(), angle.cos());

                let mut closest_point = Vector2::zero();
                point_line_distance_unclamped(position.clone(), position.clone() + dir.perp(), body.position, Some(&mut closest_point));

                let delta = closest_point - body.position;

                let c = delta.mag();

                let bias = (strength / delta_timestep) * c;

                let impulse = -body.velocity.dot(&dir); 

                body.velocity += impulse * dir + bias * delta.norm();
            }
        }    
    }
    pub fn get_deps(&mut self) -> Vec<&String> {
        match self {
            Self::DistanceJoint { body_a, body_b, .. } => {
                vec![body_a, body_b]
            }
            Self::LookJoint { body_a, body_b, .. } => {
                vec![body_a, body_b]
            }
            Self::FixedJoint { body, .. } => {
                vec![body]
            }
            Self::SlideJoint { body, ..} => {
                vec![body] 
            }
        }
    }
}
