// based off of
// https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/

use super::mesh::Mesh;
use super::{intersect_tri, Intersection, Ray, Surface, Vec3, Vector, AABB};

pub fn tri_aabb([p0, p1, p2]: &[Vec3; 3]) -> AABB {
    let mut out = AABB::new(*p0, *p1);
    out.add_point(p2);
    out
}

/// Intersect a ray with an axis-aligned bounding box
pub fn intersect_aabb(aabb: &AABB, r: &Ray, hit: Option<Intersection>) -> bool {
    let (tmin, tmax) = (0..3)
        .map(|i| {
            let tx1 = (aabb.min.0[i] - r.origin.0[i]) / r.dir.0[i];
            let tx2 = (aabb.max.0[i] - r.origin.0[i]) / r.dir.0[i];
            (tx1.min(tx2), tx1.max(tx2))
        })
        .reduce(|(amin, amax), (bmin, bmax)| (amin.max(bmin), amax.min(bmax)))
        .unwrap();
    tmax >= tmin && tmax >= 0.0 && hit.map(|hit| tmin < hit.t).unwrap_or(true)
}

/*
// TODO would use this but is slightly larger than necessary
#[derive(Debug)]
enum NodeKind {
  Leaf {
    num_prims: core::num::NonZeroUsize,
    first_prim: usize,
  },
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
        assert!(!self.is_leaf());
        self.left_child() + 1
    }
    #[inline]
    fn is_leaf(&self) -> bool {
        self.num_prims > 0
    }
    /// Sets the left child of this node, and returns (first_prim, number of primitives) it had before
    #[inline]
    fn set_left_child(&mut self, left_child: usize) -> (usize, usize) {
        assert!(self.is_leaf());
        (
            std::mem::replace(&mut self.left_child_or_first_prim, left_child),
            std::mem::replace(&mut self.num_prims, 0),
        )
    }
    #[inline]
    fn left_child(&self) -> usize {
        assert!(!self.is_leaf());
        self.left_child_or_first_prim
    }
    #[inline]
    fn first_prim(&self) -> usize {
        assert!(self.is_leaf());
        self.left_child_or_first_prim
    }
    #[inline]
    fn set_prims(&mut self, first_prim: usize, num_prims: usize) {
        self.left_child_or_first_prim = first_prim;
        self.num_prims = num_prims;
    }
}

impl BVHNode {
    #[inline]
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
        let size = 2 * mesh.num_faces() + 1;
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
        root_node.num_prims = mesh.num_faces();

        s.update_node_bounds(0);
        s.subdivide(0);

        s
    }

    /// Updates a node's bounds to contain all of the triangles it encloses
    #[inline]
    fn update_node_bounds(&mut self, idx: usize) {
        let node = &mut self.nodes[idx];
        node.aabb.reset();

        let fp = node.first_prim();
        for &tri_idx in &self.tris[fp..fp + node.num_prims] {
            let [p0, p1, p2] = self.mesh.face(tri_idx).pos(self.mesh).verts;
            node.aabb.add_point(&p0);
            node.aabb.add_point(&p1);
            node.aabb.add_point(&p2);
        }
        node.aabb.expand_by(1e-3);
    }

    #[inline]
    fn midpoint_split(&self, node: &BVHNode) -> (usize, f32) {
        // which axis are we splitting along, and along this axis at what point are we splitting?
        let axis = node.aabb.largest_dimension().0;
        // Split pos strategy is just half the largest axis
        let split_pos = node.aabb.midpoint()[axis];
        (axis, split_pos)
    }

    /// Surface Area Heuristic (SAH) split of the node, may make constructing a BVH slower
    /// But should make intersection better
    #[inline]
    fn sah_split(&self, node: &BVHNode) -> (f32, usize, f32) {
        let evaluate_sah = |axis, pos| {
            let mut left_box = AABB::empty();
            let mut right_box = AABB::empty();
            let mut left_count = 0;
            let mut right_count = 0;

            let fp = node.first_prim();

            for i in fp..fp + node.num_prims {
                let ps = self.mesh.face(self.tris[i]).pos(self.mesh).verts;
                let c = self.centroids[i];
                if c[axis] < pos {
                    for p in ps {
                        left_box.add_point(&p);
                    }
                    left_count += 1;
                } else {
                    for p in ps {
                        right_box.add_point(&p);
                    }
                    right_count += 1;
                }
            }
            let cost =
                (left_count as f32) * left_box.area() + (right_count as f32) * right_box.area();
            if !cost.is_normal() {
                return f32::INFINITY;
            }
            assert!(cost >= 0.0);
            cost
        };
        let (cost, axis, pos) = (0..3)
            .map(|axis| {
                let fp = node.first_prim();
                assert_ne!(node.num_prims, 0);
                let (cost, pos) = (fp..fp + node.num_prims)
                    .map(|i| {
                        let tri_idx = &self.tris[i];
                        let pos = self.centroids[i][axis];
                        (evaluate_sah(axis, pos), pos)
                    })
                    .min_by(|(cost_a, _), (cost_b, _)| cost_a.total_cmp(cost_b))
                    .unwrap();
                (cost, axis, pos)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .unwrap();
        (cost, axis, pos)
    }

    /// Subdivide this BVH into separate nodes for some number of the triangles it encloses.
    fn subdivide(&mut self, idx: usize) {
        let node = &self.nodes[idx];
        if node.num_prims <= 2 {
            // stop if 2 or fewer triangles
            return;
        }
        let (cost, axis, split_pos) = self.sah_split(node);
        let parent_cost = node.aabb.area() * (node.num_prims as f32);
        if cost >= parent_cost {
            return;
        }

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

        let node = &mut self.nodes[idx];

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

    #[inline]
    pub fn intersects(&self, ray: &Ray) -> Option<Intersection> {
        self.intersects_node(ray, self.root_node_idx, None)
    }

    #[inline]
    /// Recursive algorithm to compute intersection with BVH nodes
    fn intersects_node(
        &self,
        ray: &Ray,
        idx: usize,
        curr_t: Option<Intersection>,
    ) -> Option<Intersection> {
        let node = &self.nodes[idx];
        if !intersect_aabb(&node.aabb, ray, curr_t) {
            return None;
        }
        if node.is_leaf() {
            (node.first_prim()..node.first_prim() + node.num_prims)
                .filter_map(|i| {
                    let tri = &self.mesh.face(self.tris[i]).pos(self.mesh).verts;
                    intersect_tri(tri, ray, 1e-8).map(|t| Intersection {
                        t,
                        face: self.tris[i],
                    })
                })
                .chain(curr_t.into_iter())
                .min_by(|a, b| a.t.partial_cmp(&b.t).unwrap())
        } else {
            let left = self.intersects_node(ray, node.left_child(), curr_t);
            let curr_nearest = [left, curr_t]
                .into_iter()
                .flatten()
                .min_by(|a, b| a.t.partial_cmp(&b.t).unwrap());
            let right = self.intersects_node(ray, node.right_child(), curr_nearest);
            [left, right]
                .into_iter()
                .flatten()
                .min_by(|a, b| a.t.partial_cmp(&b.t).unwrap())
        }
    }

    /// Finds triangles which intersect this aabb, pushing their indeces onto out
    #[inline]
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

impl Surface for BVH<'_> {
    #[inline]
    fn intersect_ray(&self, ray: &Ray) -> Option<Intersection> {
        self.intersects(ray)
    }
}
