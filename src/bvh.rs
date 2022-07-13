// based off of
// https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/

use super::mesh::Mesh;
use super::{Vec3, Vector, AABB};

/// A ray with an origin, direction, and a length
#[derive(Debug, Clone, PartialEq)]
pub struct Ray {
    origin: Vec3,
    dir: Vec3,
}

pub fn intersect_tri([p0, p1, p2]: &[Vec3; 3], r: &Ray, eps: f32) -> Option<f32> {
    let e0 = *p1 - *p0;
    let e1 = *p2 - *p0;
    let h = r.dir.cross(&e1);
    let a = e0.dot(&h);
    if -eps < a && a < eps {
        return None;
    }
    let f = 1. / a;
    let s = r.origin - *p0;
    let u = f * s.dot(&h);
    if u < 0. || u > 1. {
        return None;
    }
    let q = s.cross(&e0);
    let v = f * r.dir.dot(&q);
    if v < 0. || v > 1. {
        return None;
    }
    let t = f * e1.dot(&q);
    Some(t).filter(|t| *t > eps)
}

pub fn tri_aabb([p0, p1, p2]: &[Vec3; 3]) -> AABB {
    let mut out = AABB::new(*p0, *p1);
    out.add_point(p2);
    out
}

/// Intersect a ray with an axis-aligned bounding box
pub fn intersect_aabb(aabb: &AABB, r: &Ray, t: Option<f32>) -> bool {
    let (tmin, tmax) = (0..3)
        .map(|i| {
            let tx1 = (aabb.min.0[i] - r.origin.0[i]) / r.dir.0[i];
            let tx2 = (aabb.max.0[i] - r.origin.0[i]) / r.dir.0[i];
            (tx1.min(tx2), tx1.max(tx2))
        })
        .reduce(|(amin, amax), (bmin, bmax)| (amin.max(bmin), amax.min(bmax)))
        .unwrap();
    tmax >= tmin && tmax >= 0.0 && t.map(|t| tmin < t).unwrap_or(true)
}

/*
// TODO would use this but is slightly larger than necessary
#[derive(Debug)]
enum NodeKind {
  Leaf {
    num_prims: core::num::NonZeroUsize,
    first_prim: usize,
  }
  Internal {
    left_child: usize,
  },
}
*/

#[derive(Debug, PartialEq, Clone)]
struct BVHNode {
    aabb: AABB,
    /// Index of node of left child of this BVHNode if num_prims == 0
    /// or first primitive_index of this bvh node if num_prims > 0
    left_child_or_first_prim: usize,

    /// Number of primitives stored in this BVH node
    num_prims: usize,
}

impl BVHNode {
    /// Returns the right child of this bvh node, which is always left child + 1
    fn right_child(&self) -> usize {
        self.left_child() + 1
    }
    fn is_leaf(&self) -> bool {
        self.num_prims > 0
    }
    /// Sets the left child of this node, and returns (first_prim, number of primitives) it had before
    fn set_left_child(&mut self, left_child: usize) -> (usize, usize) {
        (
            std::mem::replace(&mut self.left_child_or_first_prim, left_child),
            std::mem::replace(&mut self.num_prims, 0),
        )
    }
    fn left_child(&self) -> usize {
        assert_eq!(self.num_prims, 0);
        self.left_child_or_first_prim
    }
    fn first_prim(&self) -> usize {
        assert!(self.num_prims > 0);
        self.left_child_or_first_prim
    }
    fn set_prims(&mut self, first_prim: usize, num_prims: usize) {
        self.left_child_or_first_prim = first_prim;
        self.num_prims = num_prims;
    }
}

impl BVHNode {
    // returns a new empty instance of a bounding volume hierarchy node.
    fn new() -> Self {
        Self {
            aabb: AABB::empty(),
            left_child_or_first_prim: 0,
            num_prims: 0,
        }
    }
}

/// A bounding volume hierarchy (BVH) for a mesh.
#[derive(Debug)]
pub struct BVH<'a> {
    mesh: &'a Mesh,
    size: usize,
    nodes: Vec<BVHNode>,
    root_node_idx: usize,
    nodes_used: usize,

    tris: Vec<usize>,
    centroids: Vec<Vec3>,
}

impl<'a> BVH<'a> {
    pub fn new(mesh: &'a Mesh) -> Self {
        let centroids = mesh
            .faces()
            .map(|f| f.pos(mesh).centroid())
            .collect::<Vec<_>>();
        let size = mesh.num_faces();
        let nodes = vec![BVHNode::new(); size];
        let tris = (0..size).collect::<Vec<_>>();
        let mut s = Self {
            mesh,
            size,
            nodes,
            root_node_idx: 0,
            nodes_used: 1,
            tris,
            centroids,
        };
        let root_node = &mut s.nodes[0];
        root_node.num_prims = size;
        s.update_node_bounds(0);
        s.subdivide(0);
        s
    }
    /// Updates a node's bounds to contain all of the triangles it encloses
    fn update_node_bounds(&mut self, idx: usize) {
        let node = &mut self.nodes[idx];
        node.aabb.reset();
        // TODO can add a layer of indirection here
        for &tri_idx in &self.tris[node.first_prim()..node.first_prim() + node.num_prims] {
            let [p0, p1, p2] = self.mesh.face(tri_idx).pos(self.mesh).verts;
            node.aabb.add_point(&p0);
            node.aabb.add_point(&p1);
            node.aabb.add_point(&p2);
        }
    }
    /// Subdivide this BVH into separate nodes for some number of the triangles it encloses.
    fn subdivide(&mut self, idx: usize) {
        let node = &mut self.nodes[idx];
        if node.num_prims <= 2 {
            // stop if 2 or fewer triangles
            return;
        }
        // which axis are we splitting along, and along this axis at what point are we splitting?
        let axis = node.aabb.largest_dimension();
        // Split pos strategy is just half the largest axis
        let split_pos = node.aabb.midpoint().0[axis];

        let mut i = node.first_prim();
        let mut j = i + node.num_prims - 1;
        while i < j {
            if self.centroids[i].0[axis] < split_pos {
                i += 1;
            } else {
                self.centroids.swap(i, j);
                self.tris.swap(i, j);
                j -= 1;
            }
        }

        let left_count = i - node.first_prim();
        if left_count == 0 || left_count == node.num_prims {
            // did not split tris at all
            return;
        }

        // Get all these so there's no borrowck conflicts
        let (node_first_prim, node_num_prims) = node.set_left_child(self.nodes_used);
        self.nodes_used += 2;
        let left_child_idx = node.left_child();
        let right_child_idx = node.right_child();

        self.nodes[left_child_idx].set_prims(node_first_prim, left_count);
        self.nodes[right_child_idx].set_prims(i, node_num_prims - left_count);

        self.update_node_bounds(left_child_idx);
        self.update_node_bounds(right_child_idx);

        self.subdivide(left_child_idx);
        self.subdivide(right_child_idx);
    }

    pub fn intersects(&self, ray: &Ray) -> Option<f32> {
        self.intersects_node(ray, self.root_node_idx, None)
    }

    fn intersects_node(&self, ray: &Ray, idx: usize, curr_t: Option<f32>) -> Option<f32> {
        let node = &self.nodes[idx];
        if !intersect_aabb(&node.aabb, ray, curr_t) {
            return None;
        }
        if node.is_leaf() {
            (node.first_prim()..node.first_prim() + node.num_prims)
                .filter_map(|i| {
                    let tri = &self.mesh.face(self.tris[i]).pos(self.mesh).verts;
                    intersect_tri(tri, ray, 1e-5)
                })
                .chain(curr_t.into_iter())
                .min_by(|a, b| a.partial_cmp(b).unwrap())
        } else {
            let left = self.intersects_node(ray, node.left_child(), curr_t);
            let right = self.intersects_node(ray, node.right_child(), option_min(left, curr_t));
            [left, right]
                .into_iter()
                .filter_map(|t| t)
                .chain(curr_t.into_iter())
                .min_by(|a, b| a.partial_cmp(b).unwrap())
        }
    }

    /// Finds triangles which intersect this aabb, pushing their indeces onto out
    pub fn intersects_aabb(&self, aabb: &AABB, out: &mut Vec<usize>) {
        self.intersects_aabb_internal(aabb, out, self.root_node_idx);
    }
    fn intersects_aabb_internal(&self, aabb: &AABB, out: &mut Vec<usize>, idx: usize) {
        let node = &self.nodes[idx];
        if !aabb.intersects(&node.aabb) {
            return;
        }
        if node.is_leaf() {
            for i in node.first_prim()..node.first_prim() + node.num_prims {
                let tri = &self.mesh.face(self.tris[i]).pos(self.mesh).verts;
                if tri_aabb(tri).intersects(aabb) {
                    // TODO this should be a reference to triangles somewhere else?
                    out.push(i);
                }
            }
        } else {
            self.intersects_aabb_internal(aabb, out, node.left_child());
            self.intersects_aabb_internal(aabb, out, node.right_child());
        }
    }
}

fn option_min(a: Option<f32>, b: Option<f32>) -> Option<f32> {
    match (a, b) {
        (None, None) => None,
        (Some(v), None) | (None, Some(v)) => Some(v),
        (Some(a), Some(b)) => Some(a.min(b)),
    }
}
