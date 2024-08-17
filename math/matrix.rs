use super::vector2::Vector2;

#[derive(Debug)]
pub struct Matrix([f64; 6]);

impl Matrix {
    pub fn vec_mul(&self, vec2: &Vector2) -> Vector2 {
        let x = self.0[0] * vec2.x + self.0[1] * vec2.y + self.0[2];
        let y = self.0[3] * vec2.x + self.0[4] * vec2.y + self.0[5];
        return Vector2 {x, y}
    }
    pub fn mat_mul(&self, other: &Matrix) -> Matrix {
        let a = self.0[0] * other.0[0] + self.0[1] * other.0[3];
        let b = self.0[0] * other.0[1] + self.0[1] * other.0[4];
        let c = self.0[0] * other.0[2] + self.0[1] * other.0[5] + self.0[2];
        
        let d = self.0[3] * other.0[0] + self.0[4] * other.0[3];
        let e = self.0[3] * other.0[1] + self.0[4] * other.0[4];
        let f = self.0[3] * other.0[2] + self.0[4] * other.0[5] + self.0[5];

        return Matrix([a, b, c, d, e, f]);
    }
    pub fn rot(&self, rad: f64) -> Matrix {
        let rotation_matrix = Matrix([rad.cos(), -rad.sin(), 0.0, rad.sin(), rad.cos(), 0.0]);
        return rotation_matrix.mat_mul(&self); 
    }
    pub fn trans(&self, vec2: Vector2) -> Matrix {
        let translate_matrix = Matrix([1.0, 0.0, vec2.x, 0.0, 1.0, vec2.y]); 
        return translate_matrix.mat_mul(&self); 
    }
    pub fn new() -> Matrix {
        return Matrix([1.0, 0.0, 0.0, 0.0, 1.0,  0.0]);
    }
    pub fn scale(&self, vec2: Vector2) -> Matrix {
        let scale_matrix = Matrix([vec2.x, 0.0, 0.0, 0.0, vec2.y, 0.0]); 
        return scale_matrix.mat_mul(&self); 
    }
}
