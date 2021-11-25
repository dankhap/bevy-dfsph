// For simulating
pub type Real = f32;
pub type Point = cgmath::Point2<Real>;
pub type Vector = cgmath::Vector2<Real>;

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

