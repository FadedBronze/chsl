use std::f64;

use crate::math::{matrix::Matrix, vector2::Vector2};

use super::rigidbody::{Collider, RigidBody};
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature="serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BoundingBox {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

impl BoundingBox {
    pub fn point_within(&self, point: Vector2) -> bool {
        point.x > self.x && point.y > self.y && point.x < self.x + self.width && point.y < self.y + self.height
    }

    pub fn debug_draw(&self, debug_lines: &mut Vec<(Vector2, Vector2)>) {
        debug_lines.push((Vector2::new(self.x, self.y), Vector2::new(self.x + self.width, self.y)));
        debug_lines.push((Vector2::new(self.x, self.y + self.height), Vector2::new(self.x + self.width, self.y + self.height)));
        debug_lines.push((Vector2::new(self.x, self.y), Vector2::new(self.x, self.y + self.height)));
        debug_lines.push((Vector2::new(self.x + self.width, self.y), Vector2::new(self.x + self.width, self.y + self.height)));
    }
    
    pub fn debug_x(&self, debug_lines: &mut Vec<(Vector2, Vector2)>) {
        debug_lines.push((Vector2::new(self.x, self.y), Vector2::new(self.x + self.width, self.y + self.height)));
        debug_lines.push((Vector2::new(self.x + self.width, self.y), Vector2::new(self.x, self.y + self.height)));
    }

    pub fn bounds_within(&self, other: &BoundingBox) -> bool {
        self.point_within(Vector2::new(other.x, other.y)) && 
        self.point_within(Vector2::new(other.x + other.width, other.y + other.height))
    }

    pub fn area(&self) -> f64 {
        self.width * self.height
    }
}

pub trait GetBounds {
    fn get_bounds(&self) -> BoundingBox;
}

impl GetBounds for RigidBody {
    fn get_bounds(&self) -> BoundingBox {
        match &self.collider {
            Collider::Circle { radius } => {
                BoundingBox { x: self.position.x-radius, y: self.position.y-radius, width: radius * 2.0, height: radius * 2.0 }
            }
            Collider::Polygon { vertices } => {
                let mut min_x = f64::INFINITY;
                let mut max_x = f64::NEG_INFINITY;
                let mut min_y = f64::INFINITY;
                let mut max_y = f64::NEG_INFINITY;

                let mut transformed_vertices = vec![];
                let transform = Matrix::new().scale(self.scale).rot(self.rotation).trans(self.position);

                for vertex in vertices.iter() {
                    transformed_vertices.push(transform.vec_mul(vertex))
                }

                for vertex in transformed_vertices {
                    if vertex.x < min_x {
                        min_x = vertex.x;
                    }
                    if vertex.x > max_x {
                        max_x = vertex.x;
                    }
                    if vertex.y < min_y {
                        min_y = vertex.y;
                    }
                    if vertex.y > max_y {
                        max_y = vertex.y;
                    }
                }

                BoundingBox { x: min_x, y: min_y, width: max_x - min_x, height: max_y - min_y }
            }
        }
    }
}
