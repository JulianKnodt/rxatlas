#![feature(let_else)]
use clap::Parser;
use image::{ImageBuffer, Rgba};
use rsatlas::{obj, Ray, Surface, Vector};

/// Renders a mesh as example
#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Name of the person to greet
    #[clap(short, long, value_parser)]
    mesh: String,

    #[clap(long, value_parser, default_value_t = 0.)]
    eye_x: f32,
    #[clap(long, value_parser, default_value_t = 0.)]
    eye_y: f32,
    #[clap(long, value_parser, default_value_t=-2.5)]
    eye_z: f32,

    #[clap(long, value_parser, default_value_t = 512)]
    res: u32,
}

pub fn main() {
    let args = Args::parse();

    let mesh = obj::parse(&args.mesh)
        .expect("Failed to parse mesh")
        .objects
        .pop()
        .unwrap();
    let mut mesh = mesh.to_mesh();
    let (shift, scale) = mesh.aabb().to_unit();
    for v in mesh.verts.iter_mut() {
        *v += shift;
        *v /= scale;
    }
    //println!("{:?}", mesh.aabb());

    let mut out = ImageBuffer::new(args.res, args.res);
    out.save("render.png").expect("Failed to save");

    let eye = Vector::new([args.eye_x, args.eye_y, args.eye_z]);
    let up = Vector::new([0., 1., 0.]);
    let fwd = -eye.normalize();
    let right = fwd.cross(&up);
    let hw = (args.res / 2) as i32;
    for x in 0..args.res {
        let i = (x as i32) - hw;
        let u = (i as f32) / (hw as f32);
        for y in 0..args.res {
            let j = (y as i32) - hw;
            let v = (j as f32) / (hw as f32);
            let r = Ray {
                origin: eye + up * v + right * u,
                dir: fwd,
            };
            let Some(hit) = mesh.intersect_ray(&r) else { continue };
            out.put_pixel(x, args.res - y, Rgba([255u8; 4]));
        }
    }
    out.save("render.png").expect("Failed to save");
}
