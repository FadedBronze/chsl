use std::{collections::HashMap, f64::consts::PI};

use crate::{math::{matrix::Matrix, vector2::Vector2}, renderer::Renderer};

use super::{bounding_box::{BoundingBox, GetBounds}, constraint::Constraint, rigidbody::{contact_points, overlapping, resolve_collision, Collider, RigidBody}, spatial_grid::SpatialGrid};


pub struct PhysicsWorld {
    grid: SpatialGrid,
    pub bodies: HashMap<String, RigidBody>,
    //debug_points: Vec<Vector2>,
    //debug_lines: Vec<(Vector2, Vector2)>,
    constraints: Vec<Box<dyn Constraint>>,
    bounds: BoundingBox,
}

impl GetBounds for PhysicsWorld {
    fn get_bounds(&self) -> BoundingBox {
        self.bounds
    }
}

impl PhysicsWorld {
    pub fn debug_render(&mut self, renderer: &mut Renderer) {
        for (_, body) in self.bodies.iter_mut() {
            let mut points = vec![];

            match &body.collider {
                Collider::Circle { radius } => {
                    let mut angle = 0.0;
                    let iterations = 16.0;

                    for _ in 0..iterations as usize {
                        angle += PI * 2.0 / iterations;
                        points.push(Vector2::new(angle.sin() * radius, angle.cos() * radius))
                    }
                }

                Collider::Polygon { vertices } => {
                    points.append(&mut vertices.clone()); 
                }
            }

            let transform = Matrix::new().scale(body.scale).rot(body.rotation);
            
            for point in points.iter_mut() {
                *point = transform.vec_mul(point);
            }
            
            let mut last = &points[points.len()-1];
            
            for i in 0..points.len() {
                let current = &points[i];
                
                renderer.set_color(0, 0, 0, 255);

                let a = *last + body.position;
                let b = *current + body.position;

                renderer.line(a.x as i32, a.y as i32, b.x as i32, b.y as i32);

                last = current;
            }
        }

        //for point in self.debug_points.iter() {
        //    canvas.set_draw_color(sdl2::pixels::Color::RGBA(255, 0, 0, 255));
        //    canvas.draw_rect(Rect::new((point.x - 2.0) as i32, (point.y - 2.0) as i32, 4.0 as u32, 4.0 as u32)).unwrap();
        //}

        //
        //for (p1, p2) in self.debug_lines.iter() {
        //    canvas.set_draw_color(sdl2::pixels::Color::RGBA(255, 0, 0, 255));
        //    canvas.draw_line(*p1, *p2).unwrap();
        //}
    }

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
            
            ////constraints
            //for constraint in self.constraints.iter_mut() {
            //    constraint.solve(&mut self.bodies, delta_timestep)
            //}
            
            self.solve_collisions();
            
            ////constraints
            //for constraint in self.constraints.iter_mut() {
            //    constraint.solve(&mut self.bodies, delta_timestep)
            //}
            
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
            //debug_lines: vec![],
            //debug_points: vec![],
        }
    }

    pub fn add_body(&mut self, key: &str, mut rigidbody: RigidBody) {
        self.grid.insert(&key.to_string(), &mut rigidbody);
        self.bodies.insert(key.to_string(), rigidbody);
    }

    pub fn add_constraint(&mut self, constraint: Box<dyn Constraint>) {
        self.constraints.push(constraint);
    }
}
