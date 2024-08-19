use core::f64;

use crate::math::{matrix::Matrix, point_line_distance::point_line_distance, vector2::Vector2};

use super::bounding_box::GetBounds;

#[derive(Debug, Clone)]
#[cfg_attr(feature="serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Collider {
    Circle {
        radius: f64,    
    },
    Polygon {
        vertices: Vec<Vector2>
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature="serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RigidBody {
    pub position: Vector2,
    pub velocity: Vector2,

    pub rotation: f64,
    pub ang_velocity: f64,
    
    pub scale: Vector2,
    pub restitution: f64,
    pub collider: Collider,
    pub static_friction: f64,
    pub dynamic_friction: f64,
    pub inv_mass: f64,
    pub inv_inertia: f64, 

    pub gravity_scale: f64,

    pub is_static: bool,
    pub deleted: bool,
}

#[derive(Debug)]
pub struct Collision<'a> {
    pub normal: Vector2,
    pub depth: f64,
    pub contact_points: Option<Vec<Vector2>>,
    pub body_a: Option<&'a mut RigidBody>,
    pub body_b: Option<&'a mut RigidBody>,
    pub flipped: bool,
}

fn polygon_circle_collision(a: &RigidBody, a_verts: &Vec<Vector2>, b: &RigidBody, b_radius: f64) -> Option<Collision<'static>> {
    let a_transform = Matrix::new().scale(a.scale).rot(a.rotation).trans(a.position);

    let mut a_verticies: Vec<Vector2> = vec![];
        
    for vertex in a_verts.iter() {
        a_verticies.push(a_transform.vec_mul(&vertex));
    }
     
    let mut smallest_depth = f64::INFINITY; 
    let mut result_normal = Vector2::zero(); 

    let mut last = a_verticies[a_verticies.len()-1];

    for i in 0..a_verticies.len() {
        let current = a_verticies[i];
        let ab = current - last;
        let normal = ab.perp().norm();

        last = current;
        
        let min_b = (b.position - normal * b_radius * b.scale.x).dot(&normal);
        let max_b = (b.position + normal * b_radius * b.scale.x).dot(&normal);

        let mut min_a = f64::INFINITY;
        let mut max_a = f64::NEG_INFINITY;

        for j in 0..a_verticies.len() {
            let vertex = a_verticies[j];
            let proj = vertex.dot(&normal);

            if proj < min_a {min_a = proj};
            if proj > max_a {max_a = proj};
        }

        let depth = (max_a - min_b).min(max_b - min_a);

        if depth < 0.0 {
            return None
        }

        if depth < smallest_depth {
            smallest_depth = depth;
            result_normal = normal;
        }
    }

    let mut closest = Vector2::zero();
    let mut closest_mag = f64::INFINITY;

    for i in 0..a_verticies.len() {
        let new_closest = a_verticies[i] - b.position;

        if new_closest.mag() < closest_mag {
            closest_mag = new_closest.mag();
            closest = new_closest;
        }
    }

    let normal = closest.norm();
    
    let min_b = (b.position - normal * b_radius * b.scale.x).dot(&normal);
    let max_b = (b.position + normal * b_radius * b.scale.x).dot(&normal);

    let mut min_a = f64::INFINITY;
    let mut max_a = f64::NEG_INFINITY;

    for j in 0..a_verticies.len() {
        let vertex = a_verticies[j];
        let proj = vertex.dot(&normal);

        if proj < min_a {min_a = proj};
        if proj > max_a {max_a = proj};
    }

    let depth = (max_a - min_b).min(max_b - min_a);

    if depth < 0.0 {
        return None
    }

    if depth < smallest_depth {
        smallest_depth = depth;
        result_normal = normal;
    }
    
    let center_delta = b.position - a.position;
 
    if center_delta.dot(&result_normal) < 0.0 {
        smallest_depth = -smallest_depth;
    }

    return Some(Collision {
        contact_points: None,
        body_a: None,
        body_b: None,
        normal: result_normal, 
        depth: smallest_depth, 
        flipped: false,
    });
}

pub fn polygon_circle_contact_points(a: &RigidBody, b: &RigidBody) -> Vec<Vector2> {
    let Collider::Polygon { vertices: ref a_verts } = a.collider else { return vec![] };
    let Collider::Circle { .. } = b.collider else { return vec![] };
    
    let a_transform = Matrix::new().scale(a.scale).rot(a.rotation).trans(a.position);

    let mut a_verticies: Vec<Vector2> = vec![];
        
    for vertex in a_verts.iter() {
        a_verticies.push(a_transform.vec_mul(&vertex));
    }

    let mut min_distance = f64::INFINITY;
    let mut min_contact = Vector2::zero();

    let mut last = *a_verticies.last().unwrap();

    for i in 0..a_verticies.len() {
        let current = a_verticies[i];

        let mut contact = Vector2::zero();
        let distance = point_line_distance(last, current, b.position, Some(&mut contact));

        if distance < min_distance {
            min_distance = distance;
            min_contact = contact;
        }

        last = current;
    }

    return vec![min_contact]
}

pub fn contact_points(a: &RigidBody, b: &RigidBody, collision: &Collision) -> Vec<Vector2> {
    match (a.collider.clone(), b.collider.clone()) {
        (Collider::Polygon { .. }, Collider::Polygon { .. }) => {
            polygon_polygon_contact_points(a, b)
        } 
        (Collider::Circle { .. }, Collider::Polygon { .. }) => {
            polygon_circle_contact_points(b, a)
        } 
        (Collider::Polygon { .. }, Collider::Circle { .. }) => {
            polygon_circle_contact_points(a, b)
        }
        (Collider::Circle { radius }, Collider::Circle { .. }) => {
           vec![a.position + collision.normal * radius * a.scale.x] 
        }
    }
}

pub fn polygon_polygon_contact_points(a: &RigidBody, b: &RigidBody) -> Vec<Vector2> {
    let Collider::Polygon { vertices: ref a_verts } = a.collider else { return vec![] };
    let Collider::Polygon { vertices: ref b_verts } = b.collider else { return vec![] };

    let a_transform = Matrix::new().scale(a.scale).rot(a.rotation).trans(a.position);
    let b_transform = Matrix::new().scale(b.scale).rot(b.rotation).trans(b.position);

    let mut a_verticies: Vec<Vector2> = vec![];
    let mut b_verticies: Vec<Vector2> = vec![];
        
    for vertex in a_verts.iter() {
        a_verticies.push(a_transform.vec_mul(&vertex));
    }
    
    for vertex in b_verts.iter() {
        b_verticies.push(b_transform.vec_mul(&vertex));
    }

    let mut min_distance = f64::INFINITY;
    let mut contact_points = vec![];

    let mut last = a_verticies[a_verticies.len() - 1];

    for i in 0..a_verticies.len() {
        let current = a_verticies[i];

        for j in 0..b_verticies.len() {
            let vertex = b_verticies[j];
            let distance = point_line_distance(last, current, vertex, None);
            let difference = distance - min_distance;

            if difference < 0.01 && difference > -0.01 {
                contact_points.push(vertex);
            } else if distance < min_distance {
                min_distance = distance;
                contact_points = vec![vertex];
            }
        }
        
        last = current;
    }

    last = b_verticies[b_verticies.len() - 1];

    for i in 0..b_verticies.len() {
        let current = b_verticies[i];

        for j in 0..a_verticies.len() {
            let vertex = a_verticies[j];
            let distance = point_line_distance(last, current, vertex, None);
            let difference = distance - min_distance;

            if difference < 0.01 && difference > -0.01 {
                contact_points.push(vertex);
            } else if distance < min_distance {
                min_distance = distance;
                contact_points = vec![vertex];
            }
        }
        
        last = current;
    }

    return if contact_points.len() > 1 { 
        contact_points.split_at(2).0.to_vec() 
    } else { 
        contact_points 
    } 
}

fn polygon_polygon_collision(a: &RigidBody, a_verts: &Vec<Vector2>, b: &RigidBody, b_verts: &Vec<Vector2>) -> Option<Collision<'static>> {
    let a_transform = Matrix::new().scale(a.scale).rot(a.rotation).trans(a.position);
    let b_transform = Matrix::new().scale(b.scale).rot(b.rotation).trans(b.position);

    let mut a_verticies: Vec<Vector2> = vec![];
    let mut b_verticies: Vec<Vector2> = vec![];
        
    for vertex in a_verts.iter() {
        a_verticies.push(a_transform.vec_mul(&vertex));
    }
    
    for vertex in b_verts.iter() {
        b_verticies.push(b_transform.vec_mul(&vertex));
    }
    
    let mut smallest_depth = f64::INFINITY; 
    let mut result_normal = Vector2::zero(); 

    let mut last = a_verticies[a_verticies.len()-1];

    for i in 0..a_verticies.len() {
        let current = a_verticies[i];
        let ab = current - last;
        let normal = ab.norm().perp();
        last = current;
     
        let mut min_a = f64::INFINITY;
        let mut min_b = f64::INFINITY;
        let mut max_a = f64::NEG_INFINITY;
        let mut max_b = f64::NEG_INFINITY;

        for j in 0..a_verticies.len() {
            let vertex = a_verticies[j];
            let proj = vertex.dot(&normal);

            if proj < min_a {min_a = proj};
            if proj > max_a {max_a = proj};
        }

        for j in 0..b_verticies.len() {
            let vertex = b_verticies[j];
            let proj = vertex.dot(&normal);

            if proj < min_b {min_b = proj};
            if proj > max_b {max_b = proj};
        }

        let depth = (max_a - min_b).min(max_b - min_a);

        if depth < 0.0 {
            return None
        }

        if depth < smallest_depth {
            smallest_depth = depth;
            result_normal = normal;
        }
    }

    last = b_verticies[b_verticies.len()-1];

    for i in 0..b_verticies.len() {
        let current = b_verticies[i];
        let delta = current - last;
        let normal = delta.norm().perp();
        last = current;
     
        let mut min_a = f64::INFINITY;
        let mut min_b = f64::INFINITY;
        let mut max_a = f64::NEG_INFINITY;
        let mut max_b = f64::NEG_INFINITY;

        for j in 0..a_verticies.len() {
            let vertex = a_verticies[j];
            let proj = vertex.dot(&normal);

            if proj < min_a {min_a = proj};
            if proj > max_a {max_a = proj};
        }

        for j in 0..b_verticies.len() {
            let vertex = b_verticies[j];
            let proj = vertex.dot(&normal);

            if proj < min_b {min_b = proj};
            if proj > max_b {max_b = proj};
        }

        let depth = (max_a - min_b).min(max_b - min_a);

        if depth < 0.0 {
            return None
        }

        if depth < smallest_depth {
            smallest_depth = depth;
            result_normal = normal;
        }
    }

    let center_delta = b.position - a.position;
    
    if center_delta.dot(&result_normal) < 0.0 {
        smallest_depth = -smallest_depth;
    }        

    //does some weird stuff sometimes
    return Some(Collision { 
        body_a: None,
        body_b: None,
        normal: result_normal, 
        depth: smallest_depth, 
        contact_points: None, 
        flipped: false,
    })
}

pub fn resolve_collision(collision: &mut Collision) -> () {
    let Collision { normal, depth, contact_points: Some(contact_points), body_a: Some(a_body), body_b: Some(b_body), .. } = collision else {return};

    {
        let overlap = *normal * *depth;

        if a_body.is_static {
            b_body.position += overlap;
        } else if b_body.is_static {
            a_body.position -= overlap;
        } else {
            a_body.position -= overlap * 0.5;
            b_body.position += overlap * 0.5;
        }
    }

    let e = (a_body.restitution + b_body.restitution) / 2.0;

    let mut impulses = vec![];

    for contact_point in contact_points.iter() {
        let a_body_to_contact_point_perp = (*contact_point - a_body.position).perp();
        let other_to_contact_point_perp = (*contact_point - b_body.position).perp();

        let a_body_contact_point_angular_velocity = a_body_to_contact_point_perp * a_body.ang_velocity;
        let other_contact_point_angular_velocity = other_to_contact_point_perp * b_body.ang_velocity;

        let relative_contact_point_velocity = (
            a_body.velocity + a_body_contact_point_angular_velocity
        ) - (
            b_body.velocity + other_contact_point_angular_velocity
        );

        let pt = -(1.0 + e) * relative_contact_point_velocity.dot(&normal);

        let denominator = normal.dot(
            &(*normal * (a_body.inv_mass + b_body.inv_mass))
        ) +  
            a_body_to_contact_point_perp.dot(&normal).powi(2) * a_body.inv_inertia + 
            other_to_contact_point_perp.dot(&normal).powi(2) * b_body.inv_inertia
        ;

        let j = pt / if denominator == 0.0 { 0.0 } else { denominator };

        impulses.push(j)
    }

    for i in 0..impulses.len() {
        let j = impulses[i] / impulses.len() as f64;
        let contact_point = contact_points[i];

        let a_body_to_contact_point_perp = (contact_point - a_body.position).perp();
        let other_to_contact_point_perp = (contact_point - b_body.position).perp();

        let impulse_normal = j * *normal;
        
        a_body.ang_velocity += a_body_to_contact_point_perp.dot(&impulse_normal) * a_body.inv_inertia;
        b_body.ang_velocity += other_to_contact_point_perp.dot(&(-1.0 * impulse_normal)) * b_body.inv_inertia;

        a_body.velocity += a_body.inv_mass * impulse_normal;
        b_body.velocity += -b_body.inv_mass * impulse_normal;
    }

    let mut friction_impulses = vec![];

    for contact_point in contact_points.iter() {
        let o = *contact_point;
        let a = a_body.position;
        let b = b_body.position;

        let bo = o - b;
        let ao = o - a;

        let va = a_body.velocity + a_body.ang_velocity * ao.perp();
        let vb = b_body.velocity + b_body.ang_velocity * bo.perp();

        let rv = va - vb;

        let tangent = normal.perp() * -1.0;
        
        let friction_force = rv.dot(&tangent) / (
            tangent.dot(&(tangent * (a_body.inv_mass + b_body.inv_mass))) +  
            bo.dot(&tangent).powi(2) * a_body.inv_inertia + 
            ao.dot(&tangent).powi(2) * b_body.inv_inertia
        );

        friction_impulses.push(friction_force);
    }
    
    for i in 0..friction_impulses.len() {
        let j = friction_impulses[i] / friction_impulses.len() as f64 * a_body.dynamic_friction * b_body.dynamic_friction;

        let contact_point = contact_points[i];

        let a_body_to_contact_point_perp = (contact_point - a_body.position).perp();
        let other_to_contact_point_perp = (contact_point - b_body.position).perp();

        let tangent = normal.perp();

        let impulse_tangent = j * tangent;
        
        a_body.ang_velocity += a_body_to_contact_point_perp.dot(&impulse_tangent) * a_body.inv_inertia;
        b_body.ang_velocity += other_to_contact_point_perp.dot(&(-1.0 * impulse_tangent)) * b_body.inv_inertia;

        a_body.velocity += a_body.inv_mass * impulse_tangent;
        b_body.velocity += -b_body.inv_mass * impulse_tangent;
    }
}

pub fn circle_circle_collision(a: &RigidBody, a_radius: f64, b: &RigidBody, b_radius: f64) -> Option<Collision<'static>> {
    let ab = b.position - a.position;
    let depth = (a_radius * a.scale.x + b_radius * b.scale.x) - ab.mag();

    if depth < 0.0 {
        None
    } else {
        Some(Collision { normal: ab.norm(), depth, body_a: None, body_b: None, contact_points: None, flipped: false })
    } 
}

pub fn overlapping(a_body: &mut RigidBody, b_body: &mut RigidBody) -> Option<Collision<'static>> {
    match (&a_body.collider.clone(), &b_body.collider.clone()) {
        (Collider::Polygon { vertices: a_verts }, Collider::Polygon { vertices: b_verts }) => {
            polygon_polygon_collision(&a_body, &a_verts, &b_body, &b_verts)
        }
        (Collider::Polygon { vertices: verts }, Collider::Circle { radius }) => {
            polygon_circle_collision(a_body, &verts, b_body, *radius)
        }
        (Collider::Circle { radius }, Collider::Polygon { vertices: verts }) => {
            let Some(mut col) = polygon_circle_collision(b_body, &verts, a_body, *radius) else { return None; };
            col.flipped = true;
            Some(col)
        }
        (Collider::Circle { radius: a_radius }, Collider::Circle { radius: b_radius }) => {
            circle_circle_collision(a_body, *a_radius, b_body, *b_radius)
        }
    }
}
 
impl RigidBody {
    pub fn update(&mut self, delta_time: f64) -> () {
        if self.is_static { 
            self.velocity = Vector2::zero();
            self.ang_velocity = 0.0;
        } else {
            self.position += self.velocity * delta_time;
            self.rotation += self.ang_velocity * delta_time;
        }
    }

    pub fn new_circle(position: Vector2, rotation: f64, radius: f64, is_static: bool) -> Self {
        Self {
            deleted: false,
            is_static,
            position,
            rotation,
            velocity: Vector2::zero(),
            ang_velocity: 0.0,
            scale: Vector2::new(1.0, 1.0),
            restitution: 0.4,
            collider: Collider::Circle { radius },
            static_friction: 0.6,
            dynamic_friction: 0.4,
            inv_mass: if is_static { 0.0 } else { 10.0 },
            inv_inertia: if is_static { 0.0 } else { 1.0 / ((1.0 / 12.0 * 1.0) * 2.0 * (40.0 as f64).powi(2)) },
            gravity_scale: 1.0,
        }
    }

    pub fn new_square(position: Vector2, rotation: f64, scale: Vector2, is_static: bool) -> Self {
        Self {
            deleted: false,
            is_static,
            position,
            rotation,
            velocity: Vector2::zero(),
            ang_velocity: 0.0,
            scale,
            restitution: 0.4,
            collider: Collider::Polygon { 
                vertices: vec![
                    Vector2::new(1.0, 1.0),
                    Vector2::new(1.0, -1.0),
                    Vector2::new(-1.0, -1.0),
                    Vector2::new(-1.0, 1.0),
                ] 
            },
            static_friction: 0.6,
            dynamic_friction: 0.4,
            inv_mass: if is_static { 0.0 } else { 10.0 },
            inv_inertia: if is_static { 0.0 } else { 1.0 / ((1.0 / 12.0 * 1.0) * 2.0 * (40.0 as f64).powi(2)) },
            gravity_scale: 1.0,
        }
    }

    pub fn within(&self, point: Vector2) -> bool {
        match &self.collider {
            Collider::Circle { radius } => {
                (self.position - point).mag() < *radius
            }
            Collider::Polygon { .. } => {
                self.get_bounds().point_within(point)
            }
        }
    }
}
