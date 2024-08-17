use std::ops::{Add, AddAssign, Mul, Sub, SubAssign};

#[derive(Clone, Copy, Debug)]
pub struct Vector2 {
    pub x: f64,
    pub y: f64, 
}

impl Mul<Vector2> for f64 {
    type Output = Vector2;

    fn mul(self, rhs: Vector2) -> Self::Output {
        return Vector2 {
            x: self * rhs.x,
            y: self * rhs.y,
        }
    }
}

impl Mul<f64> for Vector2 {
    type Output = Vector2;

    fn mul(self, rhs: f64) -> Self::Output {
        return Vector2 {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl AddAssign for Vector2 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Add for Vector2 {
    type Output = Vector2;

    fn add(self, rhs: Vector2) -> Self::Output {
        return Vector2 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl SubAssign for Vector2 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Sub for Vector2 {
    type Output = Vector2;

    fn sub(self, rhs: Vector2) -> Self::Output {
        return Vector2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Vector2 {
    pub fn new(x: f64, y: f64) -> Vector2 {
        return Vector2 { x, y }
    }
    
    pub fn news(s: f64) -> Vector2 {
        return Vector2 { x: s, y: s }
    }
    
    pub fn zero() -> Vector2 {
        return Vector2 {
            x: 0.0,
            y: 0.0
        }
    }

    pub fn perp(&self) -> Vector2 {
        return Vector2 {
            x: -self.y,
            y: self.x,
        }
    }

    pub fn dot(&self, other: &Vector2) -> f64 {
        return self.x * other.x + self.y * other.y;
    }
    
    pub fn cross(&self, other: &Vector2) -> f64 {
        return (self.x*other.y) - (self.y*other.x);
    }
    
    pub fn norm(&self) -> Vector2 {
        let magnitude = self.mag(); 

        return Vector2 {
            x: self.x / magnitude,
            y: self.y / magnitude,
        }
    }

    pub fn mag(&self) -> f64 {
        return f64::sqrt(self.x * self.x + self.y * self.y);
    }

    pub fn mag_squared(&self) -> f64 {
        return self.x * self.x + self.y * self.y;
    }
}
