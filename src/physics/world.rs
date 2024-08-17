use std::collections::HashMap;

use crate::math::vector2::Vector2;

use super::{bounding_box::{BoundingBox, GetBounds}, constraint::Constraint, rigidbody::{contact_points, overlapping, resolve_collision, RigidBody}, spatial_grid::SpatialGrid};

pub struct PhysicsWorld {
    grid: SpatialGrid,
    pub bodies: HashMap<String, RigidBody>,
    constraints: Vec<Box<dyn Constraint>>,
    bounds: BoundingBox,
}

impl GetBounds for PhysicsWorld {
    fn get_bounds(&self) -> BoundingBox {
        self.bounds
    }
}

impl PhysicsWorld {
    fn solve_collisions(&mut self) { 
        for (k, v) in self.bodies.iter_mut() {
            self.grid.update(k, v);
        }

        self.grid.potential_collisions(&mut self.bodies, |body_a, body_b| { 
           if let Some(mut collision) = overlapping(body_a, body_b) {
               collision.contact_points = Some(contact_points(body_a, body_b, &collision));

               if collision.flipped {
                   collision.body_b = Some(body_a);
                   collision.body_a = Some(body_b);
               } else {
                   collision.body_a = Some(body_a);
                   collision.body_b = Some(body_b);
               } 
               
               resolve_collision(&mut collision);
           }
        });
    }

    pub fn update(&mut self, delta_time: f64, steps: usize) {
        let delta_timestep = delta_time / steps as f64;

        for _ in 0..steps {             
            for (_, RigidBody { velocity, gravity_scale, .. }) in self.bodies.iter_mut() {
                *velocity += Vector2::new(0.0, 981.0) * delta_timestep * *gravity_scale;
            }
            
            for constraint in self.constraints.iter_mut() {
                constraint.solve(&mut self.bodies, delta_timestep)
            }
            
            self.solve_collisions();
            
            for constraint in self.constraints.iter_mut() {
                constraint.solve(&mut self.bodies, delta_timestep)
            }
            
            //update via integration
            for (_, body) in self.bodies.iter_mut() {
                body.update(delta_timestep);
            }
        }

        //clean out of bounds bodies
        for (_, body) in self.bodies.iter_mut() {
            if !self.bounds.point_within(body.position) {
                body.deleted = true;
            }
        }

        let PhysicsWorld { grid, bodies, .. } = self;
        
        let keys: Vec<String> = bodies.keys().cloned().collect();
 
        for key in keys {
            let body = bodies.get_mut(&key).unwrap();
            let deleted = body.deleted.clone();

            if deleted {
                grid.remove::<RigidBody>(&key);
                bodies.remove(&key);
            }
        }
    }

    pub fn new(bounds: BoundingBox) -> PhysicsWorld {
        return PhysicsWorld {
            bounds,
            grid: SpatialGrid::new(bounds, 14, 10),
            bodies: HashMap::new(),
            constraints: vec![],
        }
    }

    pub fn add_body(&mut self, key: &str, mut rigidbody: RigidBody) {
        self.grid.insert(&key.to_string(), &mut rigidbody);
        self.bodies.insert(key.to_string(), rigidbody);
    }

    pub fn remove_body(&mut self, key: &str) {
        let PhysicsWorld { grid, bodies, .. } = self;
        grid.remove::<RigidBody>(&key.to_string());
        bodies.remove(&key.to_string());
    }

    pub fn add_constraint(&mut self, constraint: Box<dyn Constraint>) {
        self.constraints.push(constraint);
    }
}
