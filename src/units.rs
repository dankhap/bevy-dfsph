// For simulating
pub type Real = f32;
pub type Point = cgmath::Point2<Real>;
pub type Vector = cgmath::Vector2<Real>;

pub type Point3D = cgmath::Point3<Real>;
pub type Vector3D = cgmath::Vector3<Real>;

#[derive(Copy, Clone, PartialEq, Debug, Default)]
pub struct Rect {
    pub x: Real,
    pub y: Real,
    pub w: Real,
    pub h: Real,
    
}
impl Rect {
    pub fn new(x: f32, y: f32, w: f32, h: f32) -> Self {
        Rect {
            x: x,
            y: y,
            w: w,
            h: h,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug, Default)]
pub struct Rect3D {
    pub x: Real,
    pub y: Real,
    pub z: Real,
    pub w: Real,
    pub h: Real,
    pub d: Real,
    
}
impl Rect3D {
    pub fn new(x: f32, y: f32, z: f32, w: f32, h: f32, d: f32) -> Self {
        Rect3D {
            x: x,
            y: y,
            z: z,
            w: w,
            h: h,
            d: d,
        }
    }
}

