use super::bvh::BVH;
use super::linalg::SparseMatrix;
use super::triangle::{Triangle, Triangle2, Triangle3};
use super::{intersect_tri, Intersection, Ray, Surface, Vec2, Vec3, Vector, AABB};

use std::collections::{BTreeSet, HashMap, HashSet};
use std::iter;

// A bunch of type aliases for indexing into different stuff.
// Mostly to self-document code.

pub type VertIdx = usize;
pub type VertNIdx = usize;
pub type VertTIdx = usize;
pub type FaceIdx = usize;
pub type MatIdx = usize;

/// An edge between two vertices or texture coordinates.
#[derive(Debug, Hash, Eq, PartialEq, Copy, Clone)]
pub struct Edge([usize; 2]);

impl Edge {
    #[inline]
    pub fn new(from: usize, to: usize) -> Self {
        assert_ne!(from, to, "Cannot connect a vertex to itself");
        Edge([from.min(to), from.max(to)])
    }
    #[inline]
    pub fn flipped(self) -> [usize; 2] {
        let Edge([l, h]) = self;
        [h, l]
    }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum EdgeShareKind {
    /// A boundary edge, only used by a single face
    Boundary(FaceIdx),
    /// A edge that is shared between two faces,
    /// in no particular order.
    Shared(FaceIdx, FaceIdx),
}

#[derive(Debug, PartialEq, Eq, Copy, Clone, Default)]
pub struct MeshFace {
    pub v: [VertIdx; 3],
    pub vn: Option<[VertNIdx; 3]>,
    pub vt: Option<[VertNIdx; 3]>,

    pub mat: Option<MatIdx>,
}

#[derive(Debug, Default, Clone)]
pub struct Mesh {
    // easy to check if two vertices are adjacent, hard to find
    // adjacent vertices. Maybe need to rethink this?
    edge_face: HashMap<Edge, EdgeShareKind>,

    tex_edge_face: HashMap<Edge, EdgeShareKind>,

    pub verts: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub tex_coords: Vec<Vec2>,

    pub faces: Vec<[VertIdx; 3]>,
    face_normals: Vec<[VertNIdx; 3]>,
    face_textures: Vec<[VertTIdx; 3]>,

    // TODO this is one per face, but it can be much more compressed (ranges)
    mats: Vec<usize>,
}

/// Represents one triangular face on a mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Face<T> {
    pub verts: [T; 3],
}

impl Face<Edge> {
    /// If these faces share an edge, returns the shared edge.
    /// This assumes they share at most one edge.
    pub fn shared_edge(&self, o: &Self) -> Option<Edge> {
        for s_e in self.verts.iter() {
            for o_e in o.verts.iter() {
                if s_e == o_e {
                    return Some(*s_e);
                }
            }
        }
        None
    }
}

impl MeshFace {
    pub fn pos(&self, m: &Mesh) -> Triangle3 {
        Triangle::new(self.v.map(|vi| m.verts[vi]), ())
    }
    pub fn normals(&self, m: &Mesh) -> Option<Triangle3> {
        self.vn
            .map(|vn| Triangle::new(vn.map(|vi| m.normals[vi]), ()))
    }
    pub fn tex(&self, m: &Mesh) -> Option<Triangle2> {
        self.vt
            .map(|vt| Triangle::new(vt.map(|vi| m.tex_coords[vi]), ()))
    }
    pub fn edges(&self) -> [Edge; 3] {
        let [vi0, vi1, vi2] = self.v;
        [
            Edge::new(vi0, vi1),
            Edge::new(vi1, vi2),
            Edge::new(vi0, vi2),
        ]
    }
}

impl Mesh {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn add_vertex(
        &mut self,
        v: Vec3,
        n: Option<Vec3>,
        tex: Option<Vec2>,
    ) -> (VertIdx, Option<VertNIdx>, Option<VertTIdx>) {
        let push_w_idx = |to: &mut Vec<_>, v| {
            let idx = to.len();
            to.push(v);
            idx
        };
        let push_w_idx2 = |to: &mut Vec<_>, v| {
            let idx = to.len();
            to.push(v);
            idx
        };
        (
            push_w_idx(&mut self.verts, v),
            n.map(|n| push_w_idx(&mut self.normals, n)),
            tex.map(|n| push_w_idx2(&mut self.tex_coords, n)),
        )
    }

    /// Adds a mesh face into this mesh
    /// panicking if it has a texture and some vertices do but others do not.
    pub fn add_face(&mut self, f: MeshFace) -> usize {
        let fi = self.faces.len();
        let MeshFace { v, vt, vn, mat } = f;
        self.faces.push(v);
        if let Some(vt) = vt {
            self.face_textures.push(vt);
        }
        if let Some(vn) = vn {
            self.face_normals.push(vn);
        }
        if let Some(mat) = mat {
            self.mats.push(mat);
        }
        assert!(self.faces.len() == self.face_textures.len() || self.face_textures.is_empty());
        assert!(self.faces.len() == self.face_normals.len() || self.face_normals.is_empty());
        assert!(self.faces.len() == self.mats.len() || self.mats.is_empty());

        for i in 0..3 {
            self.edge_face
                .entry(Edge::new(v[i], v[(i + 1) % 3]))
                .and_modify(|esk| match esk {
                    EdgeShareKind::Boundary(i) => {
                        assert_ne!(*i, fi);
                        *esk = EdgeShareKind::Shared(*i, fi);
                    }
                    EdgeShareKind::Shared(..) => {
                        eprintln!("TODO Handle non-manifold more than 2 shared faces");
                    }
                })
                .or_insert(EdgeShareKind::Boundary(fi));

            // Also add texture edges if they exist.
            if let Some(vt) = vt {
                self.tex_edge_face
                    .entry(Edge::new(vt[i], vt[(i + 1) % 3]))
                    .and_modify(|esk| match esk {
                        EdgeShareKind::Boundary(i) => {
                            assert_ne!(*i, fi);
                            *esk = EdgeShareKind::Shared(*i, fi);
                        }
                        EdgeShareKind::Shared(..) => {
                            eprintln!("Edge shared by more than 2 triangles in 2D");
                        }
                    })
                    .or_insert(EdgeShareKind::Boundary(fi));
            }
        }
        fi
    }

    /// Sets each vertex to the average of each of its face normals.
    pub fn apply_average_face_normals(&mut self) {
        self.normals.clear();
        self.normals.resize(self.verts.len(), Vector::new([0.; 3]));
        self.face_normals.clone_from(&self.faces);

        for i in 0..self.num_faces() {
            let vis = self.faces[i];
            let t = Triangle::new(vis.map(|vi| self.verts[vi]), ());
            let n = t.normal();
            // TODO possibly apply a weighting based on triangle face area?
            for vi in vis {
                self.normals[vi] += n;
            }
        }
        for n in self.normals.iter_mut() {
            *n = n.normalize();
        }
    }

    #[inline]
    pub fn num_faces(&self) -> usize {
        self.faces.len()
    }

    #[inline]
    pub fn face(&self, i: usize) -> MeshFace {
        MeshFace {
            v: self.faces[i],
            vn: self.face_normals.get(i).copied(),
            vt: self.face_textures.get(i).copied(),
            mat: self.mats.get(i).copied(),
        }
    }

    /// [WIP] Finds all verts at the same location of all verts using a bvh
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

    /// Returns an AABB for this mesh that exactly bounds its vertices.
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

    pub fn boundary_loop(&self) -> Vec<usize> {
        let mut boundary_edges_map = HashMap::new();
        let mut unseen = BTreeSet::new();
        for Edge([v0, v1]) in self.boundary_edges() {
            boundary_edges_map
                .entry(v0)
                .and_modify(|pv: &mut Result<(usize, usize), usize>| {
                    *pv = Ok((pv.unwrap_err(), v1))
                })
                .or_insert(Err(v1));
            boundary_edges_map
                .entry(v1)
                .and_modify(|pv: &mut Result<(usize, usize), usize>| {
                    *pv = Ok((v0, pv.unwrap_err()))
                })
                .or_insert(Err(v0));

            unseen.insert(v0);
            unseen.insert(v1);
        }
        let mut best = vec![];
        while let Some(first) = unseen.pop_first() {
            let mut v = first;
            let mut curr = vec![v];
            loop {
                let (n0, n1) = boundary_edges_map[&v].unwrap();
                v = if unseen.remove(&n0) {
                    n0
                } else if unseen.remove(&n1) {
                    n1
                } else if n0 == first || n1 == first {
                    // saw both vertices, and one of them was the first vertex we ever saw
                    break;
                } else {
                    panic!("Already saw both vertices but the loop hasn't ended?");
                };
                curr.push(v);
            }
            if curr.len() > best.len() {
                best = curr;
            }
        }
        best
    }

    /// Texture boundary edges. If there are no textures, will return an empty iterator.
    pub fn tex_boundary_edges(&self) -> impl Iterator<Item = Edge> + '_ {
        self.tex_edge_face
            .iter()
            .filter(|&(_, &v)| matches!(v, EdgeShareKind::Boundary(..)))
            .map(|(k, v)| k)
            .copied()
    }

    /*
    /// Cuts this mesh along some number of edges.
    /// This will produce two distinct components with overlapping vertices.
    ///
    /// The caller should ensure that the edges form a cycle on the mesh.
    pub fn cut<'a>(&mut self, edges: impl IntoIterator<Item = &'a Edge>) {
        let edges = edges.into_iter();
        let mut duplicated_verts: HashMap<usize, usize> = HashMap::new();
        // Steps:
        // 2. For each face on the cut, find the face which lies on one side of the cut
        // 3. assign each of its edge to the new vertices.
        for e @ &Edge([a, b]) in edges {
            let face_idx = match self.edge_face[e] {
                EdgeShareKind::Boundary(..) => panic!("Cannot cut across a boundary edge"),
                EdgeShareKind::Shared(l_face, r_face) => {
                    // TODO find a way to check which face to keep
                    l_face
                }
            };
            let mut new_face = self.face(new_face_idx);

            // TODO need to iterate over each vertex and check to see if it is a or b,
            // and clone texture, normals, etc.
            for (i, &v) in new_face.v.iter().enumerate() {
                if v == a || v == b {
                    let (vi, ni, ti) = self.add_vertex(v, new_face.vn.get(i), new_face.vt.get(i));
                    new_face.v[i] = vi;
                    new_face.vn[i] = ni;
                    new_face.vt[i] = ti;
                }
            }
        }
    }
    */

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
            .map(|i| self.face(i).pos(self).area())
            .sum()
    }
    /// Surface area, but counts flipped triangles as positive surface area as well.
    pub fn parametric_surface_area(&self) -> f32 {
        (0..self.num_faces())
            .map(|i| self.face(i).pos(self).area().abs())
            .sum()
    }
    /// Returns the total average over all vertices.
    pub fn centroid(&self) -> Vec3 {
        super::centroid(&self.verts)
    }
    /// Returns adjacent faces for face `i`.
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
        FaceComponents::new(self)
    }

    /// Extrudes this mesh in place by pushing each vertex in the direction of the average of
    /// each of its assigned vertex normals.
    pub fn extrude(&mut self, amt: f32) {
        assert_eq!(
            self.verts.len(),
            self.normals.len(),
            "Missing normals to extrude by"
        );
        for (v, vn) in self.verts.iter_mut().zip(&self.normals) {
            *v += *vn * amt;
        }
    }
    /// Returns a BVH for intersecting this mesh.
    pub fn bvh(&self) -> BVH<'_> {
        BVH::new(self)
    }

    /// Returns whether this mesh has vertex normals
    #[inline]
    pub fn has_vertex_normals(&self) -> bool {
        !self.face_normals.is_empty() && !self.normals.is_empty()
    }

    /// Iterator over texture locations, correspoding to a face, and barycentric
    /// coordinate, corresponding to a pixel in an image with a given width and height.
    ///
    /// If the mesh does not have textures, returns None.
    pub fn interior_texels(
        &self,
        w: u32,
        h: u32,
    ) -> Option<impl Iterator<Item = (Vec2<u32>, usize, Vec3)> + '_> {
        if self.tex_coords.is_empty() {
            return None;
        }
        let v = Vector::new([w as f32, h as f32]);
        let iter = self.faces().enumerate().flat_map(move |(f_i, f)| {
            let tex = f.tex(self).unwrap();
            let mut tex_tri_aabb = (tex.aabb() * v).expand_by(1.).to_u32();
            assert!(!tex_tri_aabb.is_empty());
            // just for safety move out one more
            //tex_tri_aabb.expand_by(1);
            // Here, for each 1x1 pixel in the texture's aabb,
            // check if it intersects the triangle.
            // If it does, then output it.
            tex_tri_aabb.unit_squares().filter_map(move |pixel| {
                // convert pixel back to [0-1] fp space
                let img_pix = pixel.to_f32() / v;
                // try each corner, more reliable than midpoint.
                let midpoint = img_pix.midpoint();
                let hit = iter::once(midpoint)
                    .chain(img_pix.corners().into_iter())
                    .find(|&c| tex.contains(c))?;
                Some((pixel.min, f_i, tex.barycentric_coord(hit)))
            })
        });
        Some(iter)
    }

    /// Shifts and scales this mesh to fit inside the unit box.
    ///
    /// Transforms in place.
    pub fn rescale_to_unit_aabb(&mut self) {
        let (shift, scale) = self.aabb().to_unit();
        for v in self.verts.iter_mut() {
            *v += shift;
            *v /= scale;
        }
    }

    /// Shifts and rescales this mesh to a given axis aligned bounding box.
    pub fn rescale_to_aabb(&mut self, aabb: &AABB) {
        let (shift, scale) = self.aabb().to_other(aabb);
        for v in self.verts.iter_mut() {
            *v += shift;
            *v /= scale;
        }
    }

    /// Returns the uniform laplacian energy for this mesh.
    pub fn uniform_laplacian(&self) -> SparseMatrix {
        let mut out = SparseMatrix::new([self.verts.len(); 2]);
        for f in self.faces() {
            for e in f.edges() {
                assert!(out.insert(e.0, 1.).is_none());
                assert!(out.insert(e.flipped(), 1.).is_none());
            }
        }
        for i in 0..self.verts.len() {
            assert!(out.insert([i; 2], -1.).is_none());
        }
        out
    }

    /// Adjacency Count is #adj verts for each vert, not including itself.
    pub fn adjacency_counts(&self) -> Vec<usize> {
        let mut out = vec![0; self.verts.len()];
        for f in self.faces() {
            for Edge([vi0, vi1]) in f.edges() {
                assert_ne!(vi0, vi1);
                out[vi0] += 1;
                out[vi1] += 1;
            }
        }
        out
    }
    /// Returns a matrix which has 1 for adjacent vertices, and 0 otherwise.
    pub fn adjacency_matrix(&self) -> SparseMatrix {
        let mut out = SparseMatrix::new([self.verts.len(); 2]);
        for f in self.faces() {
            for e in f.edges() {
                assert!(out.insert(e.0, 1.).is_none());
                assert!(out.insert(e.flipped(), 1.).is_none());
            }
        }
        out
    }

    /// Maps the boundary vertices of this mesh to the unit circle in R^2.
    /// It computes their position based on winding and distance to previous vert on the
    /// boundary.
    pub fn map_boundary_verts_to_circle(&self) -> impl Iterator<Item = (usize, Vec2)> {
        let bl = self.boundary_loop();
        assert!(!bl.is_empty(), "Mesh has no cuts");
        // here assumes that boundary edges are in order
        let mut lens = vec![0.; bl.len()];
        let v = &self.verts;
        for ((i, &vi), &prev_vi) in bl.iter().enumerate().skip(1).zip(bl.iter()) {
            lens[i] = lens[i - 1] + (v[vi] - v[prev_vi]).length();
        }
        let total_len = lens.last().unwrap() + (v[bl[0]] - v[bl[bl.len() - 1]]).length();
        bl.into_iter().enumerate().map(move |(i, bl)| {
            let t = std::f32::consts::TAU * lens[i] / total_len;
            let (s, c) = t.sin_cos();
            (bl, Vector::new([c, s]))
        })
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
    pub fn trace_ray(&self, face_idx: usize, bary: Vec3, smooth_normal: Vec3) -> (Vec3, Vec3) {
        let f = self.face(face_idx);
        let fp = f.pos(self);
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

#[derive(Debug, Clone, Default)]
pub struct SimpleMesh<'a> {
    verts: &'a [Vec3],
    faces: &'a [[VertIdx; 3]],
}

/// Tutte Parameterization of a mesh
pub mod tutte;

/// Method for converting this representation designed for rendering to a half-edge mesh
/// for geometry processing.
pub mod half_edge;
