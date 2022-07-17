use super::{Vec2, Vec3, Vector};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Triangle<const N: usize, T = ()> {
    pub verts: [Vector<N>; 3],
    /// Triangles may contain extra data, or they may be a pure geometric representation.
    pub data: T,
}

pub type Triangle3<T = ()> = Triangle<3, T>;
pub type Triangle2<T = ()> = Triangle<2, T>;

impl<T, const N: usize> Triangle<N, T> {
    pub fn new(verts: [Vector<N>; 3], data: T) -> Self {
        Self { verts, data }
    }
    pub fn shrink(self) -> Triangle<N, ()> {
        Triangle {
            verts: self.verts,
            data: (),
        }
    }
    pub fn map<U>(self, mut f: impl FnMut(T) -> U) -> Triangle<N, U> {
        let Triangle { verts, data } = self;
        Triangle {
            verts,
            data: f(data),
        }
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
}

impl<T> Triangle3<T> {
    /// Returns the normal of this triangle with respect to each of the edges.
    #[inline]
    pub fn normal(&self) -> Vec3 {
        let &[v0, v1, v2] = &self.verts;
        (v1 - v0).cross(&(v2 - v0))
    }
    pub fn area(&self) -> f32 {
        super::triangle3d_area(self.verts)
    }

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
    pub fn bary_to_world(&self, bary: Vec3) -> Vec3 {
        let [v0, v1, v2] = self.verts;
        let Vector([u, v, w]) = bary;
        v0 * u + v1 * v + v2 * w
    }
}

impl<T> Triangle2<T> {
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
        Vector([a0, a1, a2]) / total_area
    }
    #[inline]
    /// Gets the global position of a barycentric coordinate for this triangle
    pub fn bary_to_world(&self, Vector([u, v, w]): Vec3) -> Vec2 {
        let [v0, v1, v2] = self.verts;
        v0 * u + v1 * v + v2 * w
    }
}