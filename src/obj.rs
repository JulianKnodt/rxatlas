use super::mesh::{Mesh, MeshFace};
use super::{Vec2, Vec3, Vector};
use std::collections::HashMap;
use std::fs::File;
use std::io::{self, BufRead, BufReader};

#[derive(Default)]
pub struct Obj {
    pub objects: Vec<ObjObject>,
    pub mtls: Vec<(String, MTL)>,
}

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
    ka: Vec3,
    kd: Vec3,
    ks: Vec3,
}

impl MTL {
    fn is_empty(&self) -> bool {
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

pub fn parse(p: &str) -> io::Result<Obj> {
    let f = File::open(p)?;
    let mut buf_read = BufReader::new(f);
    let mut obj = Obj::default();
    let mut curr_obj = ObjObject::default();
    let mut curr_mtl = None;
    let pf32 = |v: &str| v.parse::<f32>().unwrap();

    for l in buf_read.lines() {
        let l = l?;
        let mut iter = l.trim().split_whitespace();
        let Some(kind) = iter.next() else { continue };
        match kind {
            // comment
            ht if ht.starts_with("#") => continue,
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
        let mut iter = l.trim().split_whitespace();
        let Some(kind) = iter.next() else { continue };
        let kind = kind.to_lowercase();
        match kind.as_str() {
            ht if ht.starts_with('#') => continue,
            "kd" | "ks" | "ka" => match [iter.next(), iter.next(), iter.next()] {
                [None, _, _] | [_, None, _] | [_, _, None] => panic!("Unsupported {kind} {l}"),
                [Some(r), Some(g), Some(b)] => {
                    let dst = match kind.as_str() {
                        "kd" => &mut curr_mtl.kd,
                        "ks" => &mut curr_mtl.ks,
                        "ka" => &mut curr_mtl.ka,
                        _ => unreachable!(),
                    };
                    *dst = Vector::new([pf32(r), pf32(g), pf32(b)]);
                }
            },
            "map_kd" | "map_ka" | "map_ks" => {
              let Some(f) = iter.next() else { panic!("Missing file from {l}"); };
              todo!()
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
}
