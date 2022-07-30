use super::mesh::{Mesh, MeshFace};
use super::{Vec3, Vector};
use std::fs::File;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::path::Path;

#[derive(Debug, Default)]
pub struct OFF {
    v: Vec<Vec3>,
    f: Vec<[usize; 3]>,
}

pub fn parse(p: impl AsRef<Path>) -> io::Result<OFF> {
    let f = File::open(p.as_ref())?;
    let buf_read = BufReader::new(f);
    let mut off = OFF::default();
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum ParseState {
        Init,
        Vert,
        Face,
        Done,
    }

    let mut state = ParseState::Init;
    let mut verts = 0;
    let mut faces = 0;
    let pusize = |s: &str| s.parse::<usize>().unwrap();
    let pf32 = |s: &str| s.parse::<f32>().unwrap();

    for (i, l) in buf_read.lines().enumerate() {
        let l = l?;
        let mut iter = l.split_whitespace();
        let Some(first) = iter.next() else { continue };
        if first.starts_with('#') {
            continue;
        }

        match state {
            ParseState::Done => panic!("Unexpected trailing content {l}"),
            ParseState::Init if first == "OFF" => {}
            ParseState::Init => {
                let Some(face_count) = iter.next() else { panic!("Missing face count in {l}") };
                let Some(e) = iter.next() else { panic!("Missing edge count in {l}") };
                verts = pusize(first);
                faces = pusize(face_count);
                pusize(e); // check that it parses
                           // TODO could allocate space for verts or faces, but not necessary
                state = ParseState::Vert;
            }
            ParseState::Vert => {
                let Some(y) = iter.next() else { panic!("Missing y in {l}") };
                let Some(z) = iter.next() else { panic!("Missing z in {l}") };
                off.v.push(Vector::new([first, y, z].map(pf32)));
                if off.v.len() == verts as usize {
                    state = ParseState::Face;
                }
            }
            ParseState::Face => {
                let Some(vi1) = iter.next() else { panic!("Missing 2nd vert idx in {l}") };
                let Some(vi2) = iter.next() else { panic!("Missing 3rd vert idx in {l}") };
                off.f.push([first, vi1, vi2].map(pusize));
                if off.f.len() == faces as usize {
                    state = ParseState::Done;
                }
            }
        }
    }

    Ok(off)
}

impl OFF {
    pub fn to_mesh(self) -> Mesh {
        let mut m = Mesh::new();
        let Self { v, f } = self;
        m.verts.extend(v);
        for v in f {
            m.add_face(MeshFace {
                v,
                vn: None,
                vt: None,
                mat: None,
            });
        }
        m
    }
    pub fn write(&self, mut dst: impl Write) -> io::Result<()> {
        dst.write_all(b"OFF\n")?;
        writeln!(dst, "{} {} {}", self.v.len(), self.f.len(), 0)?;
        dst.write_all(b"# Vertices:\n")?;
        for Vector([x, y, z]) in &self.v {
            writeln!(dst, "{x} {y} {z}")?;
        }
        dst.write_all(b"# Faces:\n")?;
        for [vi0, vi1, vi2] in &self.f {
            writeln!(dst, "{vi0} {vi1} {vi2}")?;
        }
        Ok(())
    }
}

impl Mesh {
    /// Converts this mesh to an OFF format for serializing.
    pub fn to_off(&self) -> OFF {
        OFF {
            v: self.verts.clone(),
            f: self.faces.clone(),
        }
    }
}
