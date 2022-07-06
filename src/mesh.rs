use super::{Vec2, Vec3, Vector};
use std::collections::{HashMap, HashSet};

#[derive(Debug, Hash, Eq, PartialEq, Copy, Clone)]
pub struct Edge([usize; 2]);

impl Edge {
    fn new(from: usize, to: usize) -> Self {
        Edge([from.min(to), from.max(to)])
    }
}

#[derive(Debug, Default)]
struct Mesh {
    edges: HashSet<Edge>,
    verts: Vec<Vec3>,
    normals: Vec<Vec3>,
    tex_coords: Vec<Vec2>,
    face_indices: Vec<[usize; 3]>,
}

impl Mesh {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn add_vert(&mut self, v: Vec3, n: Vec3, tex: Vec2) {
        self.verts.push(v);
        self.normals.push(n);
        self.tex_coords.push(tex);
    }
    pub fn add_face(&mut self, idxs: [usize; 3]) {
        self.face_indices.push(idxs);
        for i in 0..3 {
            self.edges.insert(Edge::new(idxs[i], idxs[(i + 1) % 3]));
        }
    }
    pub fn num_faces(&self) -> usize {
        self.face_indices.len()
    }
    pub fn face(&self, i: usize) -> [&Vec3; 3] {
        self.face_indices[i].map(move |idx| &self.verts[idx])
    }
    /// Finds all verts at the same location of all verts using a bvh
    pub fn colocated_verts_bvh(&self) -> impl Iterator<Item = usize> {
        todo!();
        [].into_iter()
    }
    /// Finds all verts at the same location by bucketing them into eps range.
    pub fn colocated_verts_hash(&self, eps: f32) -> HashMap<[u32; 3], Vec<usize>> {
        let mut out: HashMap<[u32; 3], Vec<usize>> = HashMap::new();
        for (i, v) in self.verts.iter().enumerate() {
            out.entry((*v / eps).0.map(|v| v as u32))
                .or_default()
                .push(i);
        }
        out
    }
    pub fn colocated_verts_naive(
        &self,
        eps: f32,
    ) -> impl Iterator<Item = (usize, impl Iterator<Item = usize> + '_)> + '_ {
        let eps_sq = eps * eps;
        self.verts.iter().enumerate().map(move |(i, p)| {
            let iter = self.verts.iter().enumerate().filter_map(move |(j, q)| {
                if (*p - *q).length_sq() < eps_sq {
                    Some(j)
                } else {
                    None
                }
            });
            (i, iter)
        })
    }
}
