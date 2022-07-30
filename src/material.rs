use crate::obj::MTL;
use crate::Vec3;
use image::{DynamicImage, ImageBuffer, Rgb, Rgba};
use std::ops::Mul;

type RGBAImg = ImageBuffer<Rgba<u8>, Vec<u8>>;
type RGBImg = ImageBuffer<Rgb<u8>, Vec<u8>>;
// TODO probably need to add an empty dynamic image
#[derive(Debug)]
pub struct MaterialDefn {
    kd: RGBAImg,
    ks: RGBAImg,
    normal_map: RGBImg,
    // TODO include more here?
}

impl MaterialDefn {
    #[inline]
    pub fn from_mtl(mtl: MTL) -> Self {
        let MTL {
            map_kd,
            kd,
            map_ks,
            ks,
            bump_normal,
            ..
        } = mtl;
        let to_img = |img: Option<DynamicImage>, default: Vec3| {
            if let Some(img) = img {
                img.into_rgba8()
            } else {
                let p = default
                    .homogeneous()
                    .clamp(0., 1.)
                    .mul(255.)
                    .0
                    .map(|c| c as u8);
                ImageBuffer::from_pixel(1, 1, Rgba(p))
            }
        };
        let kd = to_img(map_kd, kd);
        let ks = to_img(map_ks, ks);
        let normal_map = if let Some(bump_n) = bump_normal {
            bump_n.into_rgb8()
        } else {
            ImageBuffer::from_pixel(1, 1, Rgb([0, 0, 255]))
        };
        Self { kd, ks, normal_map }
    }
}

/// MaterialRef is a reference to a material inserted in a material bank.
pub type MatIdx = usize;

#[derive(Debug, Default)]
pub struct MaterialBank {
    materials: Vec<MaterialDefn>,
}

impl MaterialBank {
    #[inline]
    pub fn add_material(&mut self, mat_defn: MaterialDefn) -> MatIdx {
        let r = self.materials.len();
        self.materials.push(mat_defn);
        r
    }
}
