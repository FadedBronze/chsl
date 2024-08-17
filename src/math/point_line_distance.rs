use super::vector2::Vector2;

pub fn point_line_distance_unclamped(a: Vector2, b: Vector2, p: Vector2, out_contact_point: Option<&mut Vector2>) -> f64 {
    let ab = b - a;
    let ap = p - a;

    let ratio = ap.dot(&ab) / ab.mag_squared();
    let proj = ab * ratio;

    if let Some(out_contact_point) = out_contact_point {
        *out_contact_point = ratio * ab + a;
    }
   
    let direction = ap - proj;
    direction.mag()
}

pub fn point_line_distance(a: Vector2, b: Vector2, p: Vector2, out_contact_point: Option<&mut Vector2>) -> f64 {
    let ab = b - a;
    let ap = p - a;

    let ratio = ap.dot(&ab) / ab.mag_squared();
    let proj = ab * ratio;

    if let Some(out_contact_point) = out_contact_point {
        *out_contact_point = ratio.max(0.0).min(1.0) * ab + a;
    }
   
    match ratio {
        0.0..=1.0 => {
            let direction = ap - proj;
            direction.mag()
        },
        ratio if ratio > 1.0 => {
            (p - b).mag()
        },
        ratio if ratio < 0.0 => {
            ap.mag()
        },
        _ => {
            return 0.0
        }
    }
}
