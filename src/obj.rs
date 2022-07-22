use super::mesh::{Mesh, MeshFace};
use image::{open as image_open, DynamicImage};

use super::{Vec2, Vec3, Vector};
use std::collections::HashMap;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Write};

/// Represents a single OBJ file, as well as materials for that OBJ file.
#[derive(Default)]
pub struct Obj {
    pub objects: Vec<ObjObject>,
    // TODO add groups? What's the difference between groups and objects?
    pub mtls: Vec<(String, MTL)>,
}

// TODO need to implement a way to fuse a bunch of MTL files into a single super Material.

#[derive(Debug, Clone, Default)]
pub struct ObjObject {
    v: Vec<Vec3>,
    vt: Vec<Vec2>,
    vn: Vec<Vec3>,

    f: Vec<MeshFace>,
}

impl ObjObject {
    fn is_empty(&self) -> bool {
        self.v.is_empty() && self.vt.is_empty() && self.vn.is_empty() && self.f.is_empty()
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct MTL {
    pub ka: Vec3,
    pub kd: Vec3,
    pub ks: Vec3,

    /// diffuse map
    pub map_kd: Option<DynamicImage>,
    /// specular map
    pub map_ks: Option<DynamicImage>,
    /// specular map
    pub map_ka: Option<DynamicImage>,
    /// Normal Map
    pub bump_normal: Option<DynamicImage>,
}

impl MTL {
    pub fn is_empty(&self) -> bool {
        self == &Self::default()
    }
}

fn parse_face(f0: &str, f1: &str, f2: &str, mat: Option<usize>) -> MeshFace {
    let pusize = |v: &str| v.parse::<usize>().unwrap();
    let popt = |v: Option<&str>| v.and_then(|v| v.parse::<usize>().ok());

    let split_slash = |v: &str| -> (usize, Option<usize>, Option<usize>) {
        let mut iter = v.split('/');
        match [iter.next(), iter.next(), iter.next()] {
            [None, _, _] => panic!("Missing vertex index in {v}"),
            [Some(a), b, c] => (pusize(a), popt(b), popt(c)),
        }
    };
    let (v0, vt0, vn0) = split_slash(f0);
    let (v1, vt1, vn1) = split_slash(f1);
    let (v2, vt2, vn2) = split_slash(f2);
    let v = [v0 - 1, v1 - 1, v2 - 1];
    let vt = vt0
        .zip(vt1)
        .zip(vt2)
        .map(|((vt0, vt1), vt2)| [vt0 - 1, vt1 - 1, vt2 - 1]);
    let vn = vn0
        .zip(vn1)
        .zip(vn2)
        .map(|((vn0, vn1), vn2)| [vn0 - 1, vn1 - 1, vn2 - 1]);
    MeshFace { v, vt, vn, mat }
}

/// Parses a file specified by path `p`.
/// If split_by_object, will return different objects for each group.
pub fn parse(p: &str, split_by_object: bool, split_by_group: bool) -> io::Result<Obj> {
    let f = File::open(p)?;
    let mut buf_read = BufReader::new(f);
    let mut obj = Obj::default();
    let mut curr_obj = ObjObject::default();
    let mut curr_mtl = None;
    let pf32 = |v: &str| v.parse::<f32>().unwrap();

    for l in buf_read.lines() {
        let l = l?;
        let mut iter = l.split_whitespace();
        let Some(kind) = iter.next() else { continue };
        match kind {
            // comment
            ht if ht.starts_with('#') => continue,
            "v" => match [iter.next(), iter.next(), iter.next()] {
                [None, _, _] | [_, None, _] | [_, _, None] => panic!("Unsupported `v` format {l}"),
                [Some(a), Some(b), Some(c)] => {
                    curr_obj.v.push(Vector::new([pf32(a), pf32(b), pf32(c)]));
                }
            },
            "vt" => match [iter.next(), iter.next(), iter.next()] {
                [None, _, _] | [_, None, _] => panic!("Unsupported `vt` format {l}"),
                [Some(a), Some(b), _] => {
                    curr_obj.vt.push(Vector::new([pf32(a), pf32(b)]));
                }
            },
            "vn" => match [iter.next(), iter.next(), iter.next()] {
                [None, _, _] | [_, None, _] | [_, _, None] => panic!("Unsupported `vn` format {l}"),
                [Some(a), Some(b), Some(c)] => {
                    curr_obj.vn.push(Vector::new([pf32(a), pf32(b), pf32(c)]));
                }
            },
            "f" => match [iter.next(), iter.next(), iter.next(), iter.next()] {
                [Some(_), Some(_), Some(_), Some(_)] => todo!("Implement non-triangle faces"),
                [None, _, _, _] | [_, None, _, _] | [_, _, None, _] => {
                    panic!("Unsupported `f` format {l}")
                }
                [Some(a), Some(b), Some(c), None] => {
                    curr_obj.f.push(parse_face(a, b, c, curr_mtl));
                }
            },
            "g" if !split_by_group => continue,
            "g" => todo!("need to implement groups"),
            "o" if !split_by_object => continue,
            "o" => {
                let old = std::mem::take(&mut curr_obj);
                if !old.is_empty() {
                    obj.objects.push(old);
                }
            }
            "mtllib" => {
                let Some(mtl_file) = iter.next() else { panic!("Missing mtl file in {l}") };
                let mtls = parse_mtl(mtl_file)?;
                obj.mtls.extend(mtls);
            }
            "usemtl" => {
                let Some(mtl_name) = iter.next() else { panic!("Missing mtl name in {l}") };
                let Some(mtl_idx) = obj.mtls.iter().position(|mtl| mtl.0 == mtl_name) else {
                    panic!("Could not find mtl {mtl_name}");
                };
                curr_mtl = Some(mtl_idx);
            }
            _ => todo!("Unknown line {l}"),
        };
    }
    if !curr_obj.is_empty() {
        obj.objects.push(curr_obj);
    }
    Ok(obj)
}

pub fn parse_mtl(p: &str) -> io::Result<Vec<(String, MTL)>> {
    let f = File::open(p)?;
    let mut buf_read = BufReader::new(f);
    let mut curr_mtl = MTL::default();
    let mut curr_name = String::new();

    let pf32 = |v: &str| v.parse::<f32>().unwrap();
    let mut out = vec![];
    for l in buf_read.lines() {
        let l = l?;
        let mut iter = l.split_whitespace();
        let Some(kind) = iter.next() else { continue };
        let kind = kind.to_lowercase();
        match kind.as_str() {
            ht if ht.starts_with('#') => continue,
            "kd" | "ks" | "ka" => match [iter.next(), iter.next(), iter.next()] {
                [None, _, _] | [_, None, _] | [_, _, None] => panic!("Unsupported {kind} {l}"),
                [Some(r), Some(g), Some(b)] => {
                    *match kind.as_str() {
                        "kd" => &mut curr_mtl.kd,
                        "ks" => &mut curr_mtl.ks,
                        "ka" => &mut curr_mtl.ka,
                        _ => unreachable!(),
                    } = Vector::new([pf32(r), pf32(g), pf32(b)]);
                }
            },
            "map_kd" | "map_ka" | "map_ks" => {
                let Some(f) = iter.next() else { panic!("Missing file from {l}"); };
                // TODO should be relative to mtl file?
                let img = image_open(f).expect("Failed to load mtl file");
                *match kind.as_str() {
                    "map_kd" => &mut curr_mtl.map_kd,
                    "map_ks" => &mut curr_mtl.map_ks,
                    "map_ka" => &mut curr_mtl.map_ka,
                    _ => unreachable!(),
                } = Some(img);
            }
            "newmtl" => {
                let old = std::mem::take(&mut curr_mtl);
                let new_name = iter.next().expect("missing name");
                let old_name = std::mem::replace(&mut curr_name, new_name.to_string());
                if !old.is_empty() {
                    out.push((old_name, old));
                }
            }
            _ => todo!("handle {l}"),
        }
    }
    Ok(out)
}

impl MTL {
    pub fn write(&self, mut dst: impl Write) -> io::Result<()> {
        dst.write(b"# generated by rxatlas\n")?;

        dst.write(b"newmtl default_mat\n")?;

        if let Some(map_kd) = &self.map_kd {
            map_kd
                .save("texture_kd.png")
                .expect("Failed to save texture_kd.png");
            dst.write(b"map_kd texture_kd.png\n")?;
        } else {
            write!(dst, "Kd {} {} {}\n", self.kd.x(), self.kd.y(), self.kd.z())?;
        }

        if let Some(map_ks) = &self.map_ks {
            map_ks
                .save("texture_ks.png")
                .expect("Failed to save texture_ks.png");
            dst.write(b"map_ks texture_ks.png\n")?;
        } else {
            write!(dst, "Ks {} {} {}\n", self.ks.x(), self.ks.y(), self.ks.z())?;
        }

        if let Some(map_ka) = &self.map_ka {
            map_ka
                .save("texture_ka.png")
                .expect("Failed to save texture_ka.png");
            dst.write(b"map_ka texture_ka.png\n")?;
        } else {
            write!(dst, "Ka {} {} {}\n", self.ka.x(), self.ka.y(), self.ka.z())?;
        }

        if let Some(bump_normal) = &self.bump_normal {
            bump_normal
                .save("texture_n.png")
                .expect("Failed to save texture_n.png");
            dst.write(b"bump texture_n.png\n")?;
        }

        dst.write(b"Tf 1 1 1\n")?;
        dst.write(b"Ni 1\n")?;
        dst.write(b"Ns 0\n")?;

        Ok(())
    }
}

impl ObjObject {
    /// Converts from an obj object to an internal Mesh object.
    pub fn to_mesh(self) -> Mesh {
        let mut m = Mesh::new();
        let ObjObject { v, vt, vn, f } = self;
        m.verts.extend(v);
        m.tex_coords.extend(vt);
        m.normals.extend(vn);
        // here we want to check if all the verts have a texture, or if none of them have a
        // texture. Do not allow for intermediate states?
        for f in f.into_iter() {
            m.add_face(f);
        }
        m
    }
    /// Writes this obj object out to a writer
    pub fn write(&self, mut dst: impl Write) -> io::Result<()> {
        dst.write(b"mtllib mesh.mtl\n")?;
        dst.write(b"g default\n")?;

        for v in &self.v {
            let Vector([x, y, z]) = v;
            write!(dst, "v {x} {y} {z}\n")?;
        }

        for vt in &self.vt {
            let Vector([u, v]) = vt;
            write!(dst, "vt {u} {v}\n")?;
        }

        for vn in &self.vn {
            let Vector([x, y, z]) = vn;
            write!(dst, "vn {x} {y} {z}\n")?;
        }

        dst.write(b"s 1\n")?;
        dst.write(b"g mesh_1\n")?;
        dst.write(b"usemtl default_mat\n")?;

        for f in &self.f {
            dst.write(b"f ");
            let [v0, v1, v2] = f.v.map(|v| v + 1);
            match (
                f.vt.map(|vt| vt.map(|vt| vt + 1)),
                f.vn.map(|vn| vn.map(|vn| vn + 1)),
            ) {
                (None, None) => write!(dst, "{v0}// {v1}// {v2}//"),
                (None, Some([vn0, vn1, vn2])) => write!(dst, "{v0}//{vn0} {v1}//{vn1} {v2}//{vn2}"),
                (Some([vt0, vt1, vt2]), None) => write!(dst, "{v0}/{vt0}/ {v1}/{vt1}/ {v2}/{vt2}/"),
                (Some([vt0, vt1, vt2]), Some([vn0, vn1, vn2])) => {
                    write!(dst, "{v0}/{vt0}/{vn0} {v1}/{vt1}/{vn1} {v2}/{vt2}/{vn2}")
                }
            }?;
            dst.write(b"\n");
        }
        Ok(())
    }
}

impl Mesh {
    /// Converts from the internal mesh object to an Obj
    pub fn to_obj(&self) -> ObjObject {
        let mut out = ObjObject::default();
        out.v.extend(&self.verts);
        out.vt.extend(&self.tex_coords);
        out.vn.extend(&self.normals);

        out.f.extend(self.faces());
        out
    }
}
