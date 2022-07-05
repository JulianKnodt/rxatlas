use super::{Vec2, Vector};
use image::{DynamicImage, GenericImage, ImageBuffer, Rgba};

#[derive(Debug)]
pub struct PointVisualizer {
    img: DynamicImage,
    pts: Vec<Vec2>,
}

impl PointVisualizer {
    pub fn new() -> Self {
        Self {
            img: DynamicImage::new_rgb8(512, 512),
            pts: vec![],
        }
    }
    pub fn save(&self, p: &str) {
        self.img.save(p);
    }
    pub fn add_point(&mut self, p: Vec2, [r, g, b]: [u8; 3]) {
        self.pts.push(p);
        let coord = p * Vector([511., 511.]);
        for x in -1..=1 {
            for y in -1..=1 {
                self.img.put_pixel(
                    (coord.x() + x as f32) as u32,
                    (coord.y() + y as f32) as u32,
                    Rgba([r, g, b, 0]),
                );
            }
        }
    }
}
