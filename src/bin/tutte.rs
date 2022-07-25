#![feature(let_else)]
use clap::Parser;
use rxatlas::{obj, point_vis::PointVisualizer};

/// Creates a tutte parameterization of a mesh.
#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Name of the person to greet
    #[clap(short, long, value_parser)]
    mesh: String,
}

pub fn main() {
    let args = Args::parse();

    let mesh = obj::parse(&args.mesh, false, false)
        .expect("Failed to parse mesh")
        .objects
        .pop()
        .unwrap();

    let mesh = mesh.to_mesh();
    let vts = mesh.tutte_parameterization();
    let mut pv = PointVisualizer::new();
    assert!(!vts.is_empty());
    println!("#vt {}, #v {}", vts.len(), mesh.verts.len());
    // TODO apply VTs to original mesh and write it out
    for vt in vts {
        pv.add_point((vt + 1.) / 2., [0, 0, 255]);
    }

    let bnd_uv = mesh.map_boundary_verts_to_circle().collect::<Vec<_>>();
    for (i, p) in bnd_uv.iter().enumerate() {
        let g = (i as f32 / bnd_uv.len() as f32) * 255.;
        pv.add_point((p.1 + 1.) / 2., [255, g as u8, 0]);
    }
    pv.save("tutte_param.png");
}
