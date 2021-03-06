#![feature(let_else)]
use clap::{ArgEnum, Parser};
use image::{self, GenericImageView, ImageBuffer, Rgba};
use rxatlas::{mesh::NormalWeight, obj, Ray, Surface, Vector};
use std::time::Instant;

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

    #[clap(arg_enum, long, value_parser, default_value_t = RenderKind::Normal)]
    kind: RenderKind,

    #[clap(long, value_parser)]
    texture: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ArgEnum)]
pub enum RenderKind {
    Silhouette,
    Barycentric,
    Normal,
    UV,
    Diffuse,
}

pub fn main() {
    let args = Args::parse();
    let start = Instant::now();

    let mesh = obj::parse(&args.mesh, false, false)
        .expect("Failed to parse mesh")
        .objects
        .pop()
        .unwrap();
    let texture = if RenderKind::Diffuse == args.kind {
        Some(
            image::open(args.texture.expect("Missing texture for diffuse"))
                .expect("Failed to open texture"),
        )
    } else {
        None
    };
    let mut mesh = mesh.to_mesh();
    mesh.rescale_to_unit_aabb();
    mesh.apply_average_face_normals(NormalWeight::Area);
    let bvh = mesh.bvh();

    let mut out = ImageBuffer::new(args.res, args.res);

    println!("[INFO]: Loaded BVH @ {:?}", start.elapsed());

    let eye = Vector::new([args.eye_x, args.eye_y, args.eye_z]);
    let up = Vector::new([0., 1., 0.]);
    let fwd = -eye.normalize();
    let right = fwd.cross(&up);
    let hw = (args.res / 2) as i32;
    for x in 0..args.res {
        let i = (x as i32) - hw;
        let u = (i as f32) / (hw as f32);
        for y in 1..=args.res {
            let j = (y as i32) - hw;
            let v = (j as f32) / (hw as f32);
            let r = Ray {
                origin: eye + up * v + right * u,
                dir: fwd,
            };
            let Some(hit) = bvh.intersect_ray(&r) else { continue };
            let face = mesh.face(hit.face);
            let pos = face.pos(&mesh);
            let bary = pos.barycentric_coord(r.at(hit.t));
            let rgb = match args.kind {
                RenderKind::Silhouette => Vector::one(),
                RenderKind::Normal => {
                    if let Some(n) = face.normals(&mesh) {
                        n.bary_to_world(bary).normalize()
                    } else {
                        pos.normal().normalize()
                    }
                }
                RenderKind::Barycentric => bary,
                RenderKind::UV => {
                    let uv = face.tex(&mesh).expect("No UV coordinates for this mesh");
                    uv.bary_to_world(bary).homogeneous()
                }
                RenderKind::Diffuse => {
                    let tex = texture.as_ref().unwrap();
                    let uv = face.tex(&mesh).expect("No UV coordinates for this mesh");
                    let uv = uv.bary_to_world(bary).clamp(0., 1.);
                    // nearest neighbor
                    let u = tex.width() as f32 * uv.u();
                    let v = tex.height() as f32 * (1. - uv.v());
                    let u = (u as u32).min(tex.width());
                    let v = (v as u32).min(tex.height());
                    let color = tex.get_pixel(u as u32, v as u32);
                    out.put_pixel(x, args.res - y, color);
                    continue;
                }
            };
            let [r, g, b] = (rgb * 255.).abs().0.map(|v| v as u8);
            out.put_pixel(x, args.res - y, Rgba([r, g, b, 255]));
        }
    }

    println!("[INFO]: Rendered @ {:?}", start.elapsed());
    out.save("render.png").expect("Failed to save");
}
