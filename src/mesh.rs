use super::bvh::BVH;
use super::{intersect_tri, Intersection, Ray, Surface, Vec2, Vec3, Vector, AABB};
use std::collections::{HashMap, HashSet};

#[derive(Debug, Hash, Eq, PartialEq, Copy, Clone)]
pub struct Edge([usize; 2]);

impl Edge {
    pub fn new(from: usize, to: usize) -> Self {
        Edge([from.min(to), from.max(to)])
    }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum EdgeShareKind {
    /// A boundary edge, only used by a single face
    Boundary(usize),
    /// A edge that is shared between two faces,
    /// in no particular order.
    Shared(usize, usize),
}

#[derive(Debug, PartialEq, Eq, Copy, Clone, Default)]
pub struct MeshFace {
    pub v: [usize; 3],
    pub vn: Option<[usize; 3]>,
    pub vt: Option<[usize; 3]>,
    pub mat: Option<usize>,
}

#[derive(Debug, Default, Clone)]
pub struct Mesh {
    edge_face: HashMap<Edge, EdgeShareKind>,

    pub verts: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub tex_coords: Vec<Vec2>,

    faces: Vec<MeshFace>,
}

/// Represents one triangular face on a mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Face<T> {
    pub verts: [T; 3],
}

impl<T> Face<T> {
    pub fn iter(&self) -> impl Iterator<Item = &'_ T> + '_ {
        self.verts.iter()
    }
    pub fn into_iter(self) -> impl Iterator<Item = T> {
        self.verts.into_iter()
    }
}

impl Face<Vec3> {
    pub fn area(&self) -> f32 {
        super::triangle3d_area(self.verts)
    }
    pub fn centroid(&self) -> Vec3 {
        let [v0, v1, v2] = self.verts;
        (v0 + v1 + v2) / 3.0
    }
    /// Average of edge midpoints weighted by edge length.
    pub fn center(&self) -> Vec3 {
        let [v0, v1, v2] = self.verts;
        let l0 = (v1 - v0).length();
        let l1 = (v2 - v1).length();
        let l2 = (v0 - v2).length();

        let total = l0 + l1 + l2;
        let p0 = (v1 + v0) * l0 / total;
        let p1 = (v2 + v1) * l1 / total;
        let p2 = (v0 + v2) * l2 / total;
        p0 + p1 + p2
    }
    pub fn barycentric_coord(&self, p: Vec3) -> Vec3 {
        let [v0, v1, v2] = self.verts;
        let a0 = super::triangle3d_area([v0, p, v1]);
        let a1 = super::triangle3d_area([v1, p, v2]);
        let a2 = super::triangle3d_area([v2, p, v0]);
        let total_area = a0 + a1 + a2;
        Vector([a0, a1, a2]) / total_area
    }
    /// Gets the global position of a barycentric coordinate for this triangle
    pub fn bary_to_world(&self, bary: Vec3) -> Vec3 {
        let [v0, v1, v2] = self.verts;
        let Vector([u, v, w]) = bary;
        v0 * u + v1 * v + v2 * w
    }
    pub fn normal(&self) -> Vec3 {
        let [v0, v1, v2] = self.verts;
        (v1 - v0).cross(&(v2 - v0)).normalize()
    }
}

impl Face<Edge> {
    /// If these faces share an edge, returns the shared edge.
    /// This assumes they share at most one edge.
    pub fn shared_edge(&self, o: &Self) -> Option<Edge> {
        for s_e in self.iter() {
            for o_e in o.iter() {
                if s_e == o_e {
                    return Some(*s_e);
                }
            }
        }
        None
    }
}

impl Face<usize> {
    pub fn pos(&self, m: &Mesh) -> Face<Vec3> {
        Face {
            verts: self.verts.map(|vi| m.verts[vi]),
        }
    }
    pub fn normals(&self, m: &Mesh) -> Face<Vec3> {
        Face {
            verts: self.verts.map(|vi| m.normals[vi]),
        }
    }
    pub fn tex(&self, m: &Mesh) -> Face<Vec2> {
        Face {
            verts: self.verts.map(|vi| m.tex_coords[vi]),
        }
    }
    pub fn edges(&self) -> Face<Edge> {
        let [vi0, vi1, vi2] = self.verts;
        Face {
            verts: [
                Edge::new(vi0, vi1),
                Edge::new(vi1, vi2),
                Edge::new(vi0, vi2),
            ],
        }
    }
}

impl MeshFace {
    pub fn pos(&self, m: &Mesh) -> Face<Vec3> {
        Face {
            verts: self.v.map(|vi| m.verts[vi]),
        }
    }
    pub fn normals(&self, m: &Mesh) -> Option<Face<Vec3>> {
        self.vn.map(|vn| Face {
            verts: vn.map(|vi| m.normals[vi]),
        })
    }
    pub fn tex(&self, m: &Mesh) -> Option<Face<Vec2>> {
        self.vt.map(|vt| Face {
            verts: vt.map(|vi| m.tex_coords[vi]),
        })
    }
    pub fn edges(&self) -> Face<Edge> {
        let [vi0, vi1, vi2] = self.v;
        Face {
            verts: [
                Edge::new(vi0, vi1),
                Edge::new(vi1, vi2),
                Edge::new(vi0, vi2),
            ],
        }
    }
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
    pub fn add_edges(&mut self, v: [usize; 3]) {
        self.add_face(MeshFace {
            v,
            vn: None,
            vt: None,
            mat: None,
        });
    }
    pub fn add_face(&mut self, f: MeshFace) {
        let fi = self.faces.len();
        self.faces.push(f);
        for i in 0..3 {
            let amt = self
                .edge_face
                .entry(Edge::new(f.v[i], f.v[(i + 1) % 3]))
                .and_modify(|esk| match esk {
                    EdgeShareKind::Boundary(i) => {
                        assert_ne!(*i, fi);
                        *esk = EdgeShareKind::Shared(*i, fi);
                    }
                    EdgeShareKind::Shared(..) => {
                        todo!("Handle more than 2 shared faces");
                    }
                })
                .or_insert(EdgeShareKind::Boundary(fi));
        }
    }
    pub fn num_faces(&self) -> usize {
        self.faces.len()
    }
    #[inline]
    pub fn face(&self, i: usize) -> MeshFace {
        self.faces[i]
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
    pub fn aabb(&self) -> AABB {
        let mut aabb = AABB::empty();
        for v in &self.verts {
            aabb.add_point(v);
        }
        aabb
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
    /// Returns edges which are boundaries of cuts
    pub fn boundary_edges(&self) -> impl Iterator<Item = Edge> + '_ {
        self.edge_face
            .iter()
            .filter(|&(_, &v)| matches!(v, EdgeShareKind::Boundary(..)))
            .map(|(k, v)| k)
            .copied()
    }
    pub fn is_boundary_edge(&self, e: &Edge) -> Option<bool> {
        self.edge_face
            .get(e)
            .map(|esk| matches!(esk, EdgeShareKind::Boundary(..)))
    }
    pub fn faces(&self) -> impl Iterator<Item = MeshFace> + '_ {
        (0..self.num_faces()).map(|i| self.face(i))
    }
    pub fn surface_area(&self) -> f32 {
        (0..self.num_faces())
            .map(|i| self.face(i).pos(&self).area())
            .sum()
    }
    /// Surface area, but counts flipped triangles as positive surface area as well.
    pub fn parametric_surface_area(&self) -> f32 {
        (0..self.num_faces())
            .map(|i| self.face(i).pos(&self).area().abs())
            .sum()
    }
    pub fn centroid(&self) -> Vec3 {
        super::centroid(&self.verts)
    }
    pub fn adjacent_faces(&self, i: usize) -> impl Iterator<Item = usize> + '_ {
        self.face(i)
            .edges()
            .into_iter()
            .filter_map(move |e| match self.edge_face[&e] {
                EdgeShareKind::Boundary(o) => {
                    assert_eq!(o, i);
                    None
                }
                EdgeShareKind::Shared(a, b) => match (a == i, b == i) {
                    (true, true) => panic!("Edge shares face with self"),
                    (false, false) => panic!("Edge does not share face with original"),
                    (false, true) => Some(a),
                    (true, false) => Some(b),
                },
            })
    }
    /// Returns a map of face index to face's connected component.
    pub fn face_components(&self) -> FaceComponents {
        FaceComponents::new(&self)
    }

    /// Extrudes this mesh in place by pushing each vertex in the direction of the average of
    /// each of its normals.
    pub fn extrude(&mut self, amt: f32) {
        let mut cnts = vec![0; self.verts.len()];
        for f in self.faces() {
            for vi in f.v.into_iter() {
                cnts[vi] += 1;
            }
        }
        let mut extrude_amt = vec![Vector::<3>::zero(); self.verts.len()];
        for f in self.faces() {
            let n = f.pos(&self).normal();
            for vi in f.v.into_iter() {
                extrude_amt[vi] += n / (cnts[vi] as f32);
            }
        }
        for (ex_amt, v) in extrude_amt.into_iter().zip(self.verts.iter_mut()) {
            *v += ex_amt.normalize();
        }
    }
    pub fn bvh(&self) -> BVH<'_> {
        BVH::new(self)
    }
}

#[derive(Debug)]
pub struct FaceComponents {
    /// Groups of faces, with their index as the value of a face.
    /// A "group" is just a connected component.
    /// Key is face number, value is connected component number.
    pub assignments: HashMap<usize, u32>,
}

impl FaceComponents {
    fn new(m: &Mesh) -> Self {
        let mut curr_f = 0;
        let mut assignments = HashMap::new();

        let mut work = vec![];
        let mut curr_group = 0;
        while curr_f < m.num_faces() {
            work.push(curr_f);
            curr_f += 1;
            let mut any = false;
            while let Some(f) = work.pop() {
                if assignments.contains_key(&f) {
                    continue;
                }
                any = true;
                assignments.insert(f, curr_group);
                for n_f in m.adjacent_faces(f) {
                    work.push(n_f);
                }
            }
            curr_group += any as u32;
        }
        FaceComponents { assignments }
    }
}

// https://alain.xyz/research/baked-texture-generation/assets/gpuzen2-baked-texture-generation.pdf
// implementation of baking components
impl Mesh {
    fn trace_ray(&self, face_idx: usize, bary: Vec3, smooth_normal: Vec3) -> (Vec3, Vec3) {
        let f = self.face(face_idx);
        let fp = f.pos(&self);
        let pos = fp.bary_to_world(bary);
        let dir = -fp.normal();
        (pos, dir)
    }
}

impl Surface for Mesh {
    fn intersect_ray(&self, r: &Ray) -> Option<Intersection> {
        let mut out = None;
        for (i, f) in self.faces().enumerate() {
            let p = f.pos(self);
            let int = intersect_tri(&p.verts, r, 1e-5);
            out = match (out, int) {
                (prev, None) => prev,
                (None, Some(t)) => Some(Intersection { face: i, t }),
                (Some(prev), Some(t)) => Some(if prev.t <= t {
                    prev
                } else {
                    Intersection { face: i, t }
                }),
            };
        }
        out
    }
}
