use super::bvh::BVH;
use super::triangle::{Triangle, Triangle2, Triangle3};
use super::{intersect_tri, Intersection, Ray, Surface, Vec2, Vec3, Vector, AABB};
use std::collections::{HashMap, HashSet};
use std::iter;

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
    tex_edge_face: HashMap<Edge, EdgeShareKind>,

    pub verts: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub tex_coords: Vec<Vec2>,

    faces: Vec<[usize; 3]>,
    face_normals: Vec<[usize; 3]>,
    face_textures: Vec<[usize; 3]>,

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

    /// Adds a mesh face into this mesh
    /// panicking if it has a texture and some vertices do but others do not.
    pub fn add_face(&mut self, f: MeshFace) {
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
                        todo!("Handle more than 2 shared faces");
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
                            panic!("Edge shared by more than 2 triangles in 2D");
                        }
                    })
                    .or_insert(EdgeShareKind::Boundary(fi));
            }
        }
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

    pub fn tex_boundary_edges(&self) -> impl Iterator<Item = Edge> + '_ {
        self.tex_edge_face
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
            .map(|i| self.face(i).pos(self).area())
            .sum()
    }
    /// Surface area, but counts flipped triangles as positive surface area as well.
    pub fn parametric_surface_area(&self) -> f32 {
        (0..self.num_faces())
            .map(|i| self.face(i).pos(self).area().abs())
            .sum()
    }
    pub fn centroid(&self) -> Vec3 {
        super::centroid(&self.verts)
    }
    pub fn adjacent_faces(&self, i: usize) -> impl Iterator<Item = usize> + '_ {
        self.face(i)
            .edges()
            .verts
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
                let (count_hits, pt_sum) = iter::once(midpoint)
                    .chain(img_pix.corners().into_iter())
                    .filter(|&c| tex.contains(c))
                    .map(|c| (1, c))
                    .reduce(|(e0, c0), (e1, c1)| (e0 + e1, c0 + c1))?;
                // TODO maybe return an iterator which filters items within the triangle?
                assert_ne!(count_hits, 0);
                Some((
                    pixel.min,
                    f_i,
                    tex.barycentric_coord(pt_sum / (count_hits as f32)),
                ))
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
    /*
    /// Returns the assignment of each location in a texture to a face.
    /// If there are no texture coordinates for a mesh, returns an empty vec.
    pub fn tex_faces(&self, w: u32, h: u32) -> Vec<Vec<u32>> {
        if self.tex_coords.is_empty() {
            return vec![];
        }
        let mut out = vec![vec![u32::MAX; w as usize]; h as usize];
        for f in self.faces() {
            let t = f.tex(&self).unwrap();
            // simple approach, just scan through bounding box and see if it's inside the triangle
            let aabb = t.aabb();
        }
        out
    }
    */

    /*
    // TODO before implementing this need to have some way to represent adjacencies.
    pub fn laplacian_weights(&self, from: usize, to: usize) -> f32 {
      if from == to {
        todo!();
      } else {
        if self.edge_face.contains_key(&Edge::new(from, to)) {
          1.
        } else {
          0.
        }
      }
    }
    */
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
