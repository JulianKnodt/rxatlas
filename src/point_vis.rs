/// Small tool for visualizing a set of 2D points.
use super::{Vec2, Vector};
use image::{DynamicImage, GenericImage, GenericImageView, ImageBuffer, Rgba};

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
                let fx = (coord.x() + x as f32) as u32;
                let fy = (coord.y() + y as f32) as u32;
                if !self.img.in_bounds(fx, fy) {
                    continue;
                }
                self.img.put_pixel(fx, fy, Rgba([r, g, b, 255]));
            }
        }
    }
}

impl Default for PointVisualizer {
    fn default() -> Self {
        Self::new()
    }
}
