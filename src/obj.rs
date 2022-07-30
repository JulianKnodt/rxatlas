use super::mesh::{Mesh, MeshFace};
use image::{self, DynamicImage};

use super::{Vec2, Vec3, Vector};
use std::collections::HashMap;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Write};
use std::path::Path;

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
    pub ke: Vec3,

    /// diffuse map
    pub map_kd: Option<DynamicImage>,
    /// specular map
    pub map_ks: Option<DynamicImage>,
    /// specular map
    pub map_ka: Option<DynamicImage>,
    /// Normal Map
    pub bump_normal: Option<DynamicImage>,

    /// Bump/Height Map
    pub disp: Option<DynamicImage>,

    /// Ambient Occlusion Map
    pub map_ao: Option<DynamicImage>,

    /// Read but don't do anything yet
    pub ns: f32,
    pub ni: f32,
    pub d: f32,

    pub illum: u8,
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
pub fn parse(p: impl AsRef<Path>, split_by_object: bool, split_by_group: bool) -> io::Result<Obj> {
    let f = File::open(p.as_ref())?;
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
            // TODO not sure what to do for smoothing groups
            "s" => {}
            "mtllib" => {
                let Some(mtl_file) = iter.next() else { panic!("Missing mtl file in {l}") };
                let mtls = parse_mtl(p.as_ref().with_file_name(mtl_file))?;
                obj.mtls.extend(mtls);
            }
            "usemtl" => {
                let Some(mtl_name) = iter.next() else { panic!("Missing mtl name in {l}") };
                let Some(mtl_idx) = obj.mtls.iter().position(|mtl| mtl.0 == mtl_name) else {
                    eprintln!("Could not find mtl {mtl_name}, have {:?}",
                        obj.mtls.iter().map(|(n, _)| n).collect::<Vec<_>>());
                    continue;
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

pub fn parse_mtl(p: impl AsRef<Path>) -> io::Result<Vec<(String, MTL)>> {
    let f = File::open(p.as_ref())?;
    let buf_read = BufReader::new(f);
    let mut curr_mtl = MTL::default();
    let mut curr_name = String::new();

    let pf32 = |v: &str| v.parse::<f32>().unwrap();
    let pu8 = |v: &str| v.parse::<u8>().unwrap();
    let mut out = vec![];
    for l in buf_read.lines() {
        let l = l?;
        let mut iter = l.split_whitespace();
        let Some(kind) = iter.next() else { continue };
        let kind = kind.to_lowercase();
        match kind.as_str() {
            ht if ht.starts_with('#') => continue,
            "kd" | "ks" | "ka" | "ke" => match [iter.next(), iter.next(), iter.next()] {
                [None, _, _] | [_, None, _] | [_, _, None] => panic!("Unsupported {kind} {l}"),
                [Some(r), Some(g), Some(b)] => {
                    *match kind.as_str() {
                        "kd" => &mut curr_mtl.kd,
                        "ks" => &mut curr_mtl.ks,
                        "ka" => &mut curr_mtl.ka,
                        "ke" => &mut curr_mtl.ke,
                        _ => unreachable!(),
                    } = Vector::new([pf32(r), pf32(g), pf32(b)]);
                }
            },
            "map_kd" | "map_ka" | "map_ks" | "disp" | "bump_normal" | "map_normal" | "map_ao" => {
                let Some(f) = iter.next() else { panic!("Missing file from {l}"); };
                // TODO should be relative to mtl file?
                let mtl_path = p.as_ref().with_file_name(f);
                let img = image::open(mtl_path).expect("Failed to load mtl file");
                *match kind.as_str() {
                    "map_kd" => &mut curr_mtl.map_kd,
                    "map_ks" => &mut curr_mtl.map_ks,
                    "map_ka" => &mut curr_mtl.map_ka,
                    "disp" => &mut curr_mtl.disp,
                    "map_ao" => &mut curr_mtl.map_ao,
                    "map_normal" | "bump_normal" => &mut curr_mtl.bump_normal,
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
            "ns" | "ni" | "d" => {
                let Some(v) = iter.next() else { panic!("Missing value in {l}") };
                *match kind.as_str() {
                    "ni" => &mut curr_mtl.ni,
                    "ns" => &mut curr_mtl.ns,
                    "d" => &mut curr_mtl.d,
                    _ => unreachable!(),
                } = pf32(v);
            }
            "illum" => {
                let Some(n) =  iter.next() else { panic!("Missing value in {l}") };
                curr_mtl.illum = pu8(n);
            }
            _ => todo!("TODO implement MTL line {l}"),
        }
    }
    if !curr_mtl.is_empty() {
        out.push((curr_name, curr_mtl));
    }
    Ok(out)
}

impl MTL {
    pub fn write(&self, mut dst: impl Write, name_prefix: &str) -> io::Result<()> {
        dst.write_all(b"# generated by rxatlas\n")?;
        dst.write_all(b"newmtl default_mat\n")?;

        if let Some(map_kd) = &self.map_kd {
            let name = format!("{name_prefix}_kd.png");
            map_kd
                .save(&name)
                .unwrap_or_else(|_| panic!("Failed to save {name}"));
            writeln!(dst, "map_kd {name}")?;
        }
        writeln!(dst, "Kd {} {} {}", self.kd.x(), self.kd.y(), self.kd.z())?;

        if let Some(map_ks) = &self.map_ks {
            let name = format!("{name_prefix}_ks.png");
            map_ks
                .save(&name)
                .unwrap_or_else(|_| panic!("Failed to save {name}"));
            writeln!(dst, "map_ks {name}")?;
        }
        writeln!(dst, "Ks {} {} {}", self.ks.x(), self.ks.y(), self.ks.z())?;

        if let Some(map_ka) = &self.map_ka {
            let name = format!("{name_prefix}_ks.png");
            map_ka
                .save(&name)
                .unwrap_or_else(|_| panic!("Failed to save {name}"));
            writeln!(dst, "map_ka {name}")?;
        }
        writeln!(dst, "Ka {} {} {}", self.ka.x(), self.ka.y(), self.ka.z())?;

        writeln!(dst, "Ke {} {} {}", self.ke.x(), self.ke.y(), self.ke.z())?;

        if let Some(bump_normal) = &self.bump_normal {
            let name = format!("{name_prefix}_n.png");
            bump_normal
                .save(&name)
                .unwrap_or_else(|_| panic!("Failed to save {name}"));
            writeln!(dst, "bump {name}")?;
        }

        if let Some(disp) = &self.disp {
            let name = format!("{name_prefix}_displacement.png");
            disp.save(&name)
                .unwrap_or_else(|_| panic!("Failed to save {name}"));
            writeln!(dst, "disp {name}")?;
        }

        if let Some(map_ao) = &self.map_ao {
            let name = format!("{name_prefix}_ao.png");
            map_ao
                .save(&name)
                .unwrap_or_else(|_| panic!("Failed to save {name}"));
            writeln!(dst, "map_ao {name}")?;
        }

        writeln!(dst, "Ns {}", self.ns)?;
        writeln!(dst, "Ni {}", self.ni)?;
        writeln!(dst, "d {}", self.d)?;
        writeln!(dst, "illum {}", self.illum)?;

        Ok(())
    }
}

impl ObjObject {
    /// Converts from an obj object to an internal Mesh object.
    pub fn to_mesh(self) -> Mesh {
        let mut m = Mesh::new();
        let ObjObject { v, vt, vn, f } = self;
        m.verts.extend(v);
        m.normals.extend(vn);
        m.tex_coords.extend(vt);
        // here we want to check if all the verts have a texture, or if none of them have a
        // texture. Do not allow for intermediate states?
        for f in f.into_iter() {
            m.add_face(f);
        }
        m
    }
    /// Writes this obj object out to a writer
    pub fn write(&self, mut dst: impl Write, mtl_name: Option<&str>) -> io::Result<()> {
        if let Some(mtl_name) = mtl_name {
            writeln!(dst, "mtllib {}", mtl_name)?;
        }
        dst.write_all(b"g default\n")?;

        for v in &self.v {
            let Vector([x, y, z]) = v;
            writeln!(dst, "v {x} {y} {z}")?;
        }

        for vt in &self.vt {
            let Vector([u, v]) = vt;
            writeln!(dst, "vt {u} {v}")?;
        }

        for vn in &self.vn {
            let Vector([x, y, z]) = vn;
            writeln!(dst, "vn {x} {y} {z}")?;
        }

        dst.write_all(b"s 1\n")?;
        dst.write_all(b"g mesh_1\n")?;
        dst.write_all(b"usemtl default_mat\n")?;

        for f in &self.f {
            dst.write_all(b"f ")?;
            let [v0, v1, v2] = f.v.map(|v| v + 1);
            match (
                f.vt.map(|vt| vt.map(|vt| vt + 1)),
                f.vn.map(|vn| vn.map(|vn| vn + 1)),
            ) {
                (None, None) => writeln!(dst, "{v0}// {v1}// {v2}//"),
                (None, Some([vn0, vn1, vn2])) => {
                    writeln!(dst, "{v0}//{vn0} {v1}//{vn1} {v2}//{vn2}")
                }
                (Some([vt0, vt1, vt2]), None) => {
                    writeln!(dst, "{v0}/{vt0}/ {v1}/{vt1}/ {v2}/{vt2}/")
                }
                (Some([vt0, vt1, vt2]), Some([vn0, vn1, vn2])) => {
                    writeln!(dst, "{v0}/{vt0}/{vn0} {v1}/{vt1}/{vn1} {v2}/{vt2}/{vn2}")
                }
            }?;
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
