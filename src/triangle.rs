use super::{Extent, Vec2, Vec3, Vector};
use std::array::from_fn;
use std::ops::Add;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Triangle<const N: usize, T = ()> {
    pub verts: [Vector<N>; 3],
    /// Triangles may contain extra data, or they may be a pure geometric representation.
    pub data: T,
}
impl<const N: usize, T> Add<Vector<N>> for Triangle<N, T> {
    type Output = Self;
    fn add(self, o: Vector<N>) -> Self {
        let Self { verts, data } = self;
        let verts = verts.map(|v| v + o);
        Self { verts, data }
    }
}

pub type Triangle3<T = ()> = Triangle<3, T>;
pub type Triangle2<T = ()> = Triangle<2, T>;
impl<T: Default> Default for Triangle3<T> {
    fn default() -> Self {
        Triangle3 {
            verts: Default::default(),
            data: T::default(),
        }
    }
}

impl<T, const N: usize> Triangle<N, T> {
    #[inline]
    pub const fn new(verts: [Vector<N>; 3], data: T) -> Self {
        Self { verts, data }
    }

    /// Splits this triangle into its data and its vertices, returning a tuple of a (triangle with no
    /// data, and the old associated data).
    #[inline]
    pub fn shrink(self) -> (Triangle<N, ()>, T) {
        let Triangle { verts, data } = self;
        (Triangle { verts, data: () }, data)
    }

    #[inline]
    pub fn map<U>(self, mut f: impl FnMut(&Triangle<N, ()>, T) -> U) -> Triangle<N, U> {
        let Triangle { verts, data } = self;
        let data = f(&Triangle::new(verts, ()), data);
        Triangle { verts, data }
    }

    /// Average of all vertices
    pub fn centroid(&self) -> Vector<N> {
        let &[v0, v1, v2] = &self.verts;
        (v0 + v1 + v2) / 3.0
    }

    /// Average of edge midpoints weighted by edge length.
    pub fn center(&self) -> Vector<N> {
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
    pub fn aabb(&self) -> Extent<N> {
        let [p0, p1, p2] = self.verts;
        let mut out = Extent::new(p0, p1);
        out.add_point(&p2);
        out
    }
}

impl<T> Triangle3<T> {
    /// Returns the normal of this triangle with respect to each of the edges.
    #[inline]
    pub fn normal(&self) -> Vec3 {
        let &[v0, v1, v2] = &self.verts;
        (v1 - v0).cross(&(v2 - v0)).normalize()
    }
    #[inline]
    pub fn area(&self) -> f32 {
        super::triangle3d_area(self.verts)
    }

    #[inline]
    pub fn barycentric_coord(&self, p: Vec3) -> Vec3 {
        let [v0, v1, v2] = self.verts;
        let a0 = super::triangle3d_area([v0, p, v1]);
        let a1 = super::triangle3d_area([v1, p, v2]);
        let a2 = super::triangle3d_area([v2, p, v0]);
        let total_area = a0 + a1 + a2;
        // This order follows the last vertex in the set
        Vector([a1, a2, a0]) / total_area
    }
    /// Gets the global position of a barycentric coordinate for this triangle
    #[inline]
    pub fn bary_to_world(&self, bary: Vec3) -> Vec3 {
        let [v0, v1, v2] = self.verts;
        let Vector([u, v, w]) = bary;
        v0 * u + v1 * v + v2 * w
    }

    /// Returns whether or not the triangular prism defined by this triangle contains a point.
    #[inline]
    pub fn contains(&self, v: Vec3) -> bool {
        self.barycentric_coord(v)
            .0
            .iter()
            .all(|&c| 0. <= c && c <= 1.)
    }

    /// Returns a basis of vectors, [Tangent, Bitangent, Normal]
    /// With the provided normal.
    pub fn tbn(&self, uv: &Triangle2, n: Vec3) -> [Vec3; 3] {
        let &[v0, v1, v2] = &self.verts;
        assert!(self.area() > 0.);
        let e0 = v1 - v0;
        let e1 = v2 - v0;

        let &[uv0, uv1, uv2] = &uv.verts;
        // handle flipped uv triangles.
        let (duv0, duv1) = (uv1 - uv0, uv2 - uv0);
        let f = duv0.cross(&duv1).recip();
        let tan = Vector(from_fn(|dim| duv1.y() * e0[dim] - duv0.y() * e1[dim])) * f;
        // Create orthonormal basis
        let tan = tan.normalize();
        let bit = n.cross(&tan).normalize();
        let tan = bit.cross(&n).normalize();

        assert!(tan.dot(&n).abs() < 1e-4);
        assert!(bit.dot(&n).abs() < 1e-4);
        assert!(tan.dot(&bit).abs() < 1e-4);
        // Flip basis if the UV triangle is reversed.
        // important for continuity.
        let [tan, bit] = if uv.area() <= 0.0 {
            [bit, tan]
        } else {
            [tan, bit]
        };
        [tan, bit, n]
    }
}

impl<T> Triangle2<T> {
    /// Returns the area of this triangle
    #[inline]
    pub fn area(&self) -> f32 {
        super::triangle2d_area(self.verts)
    }
    pub fn barycentric_coord(&self, p: Vec2) -> Vec3 {
        let [v0, v1, v2] = self.verts;
        let a0 = super::triangle2d_area([v0, p, v1]);
        let a1 = super::triangle2d_area([v1, p, v2]);
        let a2 = super::triangle2d_area([v2, p, v0]);
        let total_area = a0 + a1 + a2;
        Vector([a1, a2, a0]) / total_area
    }
    #[inline]
    /// Gets the global position of a barycentric coordinate for this triangle
    pub fn bary_to_world(&self, Vector([u, v, w]): Vec3) -> Vec2 {
        let [v0, v1, v2] = self.verts;
        v0 * u + v1 * v + v2 * w
    }

    #[inline]
    pub fn contains(&self, v: Vec2) -> bool {
        self.barycentric_coord(v)
            .0
            .iter()
            .all(|&c| 0. <= c && c <= 1.)
    }
}
