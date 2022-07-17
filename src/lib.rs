#![feature(let_else)]
#![allow(unused)]
use std::array::from_fn;

mod bvh;
pub mod point_vis;
pub mod rand;
pub mod triangle;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Vector<const N: usize, T = f32>(pub [T; N]);
pub type Vec2 = Vector<2>;
pub type Vec3 = Vector<3>;
impl<const N: usize, T> Vector<N, T> {
    pub fn new(v: [T; N]) -> Self {
        Self(v)
    }
}

impl<const N: usize> Vector<N> {
    /// Creates a zeroed vector.
    #[inline]
    pub fn zero() -> Self {
        Self([0.; N])
    }
    /// Creates a vector with only ones
    #[inline]
    pub fn one() -> Self {
        Self([1.; N])
    }
    /// Computes the dot product of two vectors.
    pub fn dot(&self, o: &Self) -> f32 {
        self.0
            .iter()
            .zip(o.0.iter())
            .map(|(l, r)| l * r)
            .sum::<f32>()
    }
    pub fn length_sq(&self) -> f32 {
        self.dot(self)
    }
    pub fn length(&self) -> f32 {
        self.dot(self).sqrt()
    }
    pub fn normalize(&self) -> Self {
        (*self) / (self.length() + 1e-6)
    }
    pub fn approx_equals(&self, o: &Self, eps: f32) -> bool {
        (*self - *o).length() < eps
    }
    pub fn min(&self, o: &Self) -> Self {
        Self(from_fn(|i| self.0[i].min(o.0[i])))
    }
    pub fn max(&self, o: &Self) -> Self {
        Self(from_fn(|i| self.0[i].max(o.0[i])))
    }
    pub fn clamp(&self, min: f32, max: f32) -> Self {
        Self(from_fn(|i| self.0[i].min(max).max(min)))
    }
    pub fn is_finite(&self) -> bool {
        self.0.iter().copied().all(f32::is_finite)
    }
    pub fn abs(&self) -> Self {
        Self(self.0.map(|v| v.abs()))
    }
    pub fn cast<T>(&self) -> [T; N]
    where
        f32: Into<T>,
    {
        self.0.map(|v| v.into())
    }
}

impl Vec2 {
    #[inline]
    pub fn x(&self) -> f32 {
        self.0[0]
    }
    #[inline]
    pub fn y(&self) -> f32 {
        self.0[1]
    }
    #[inline]
    pub fn u(&self) -> f32 {
        self.0[0]
    }
    #[inline]
    pub fn v(&self) -> f32 {
        self.0[1]
    }
    #[inline]
    pub fn cross(&self, other: &Self) -> f32 {
        self.x() * other.y() - self.y() * other.x()
    }
    /// Gets an orthogonal vector to this vector
    #[inline]
    pub fn ortho(&self) -> Vec2 {
        Vector([-self.y(), self.x()])
    }
    #[inline]
    pub fn homogeneous(&self) -> Vec3 {
        Vector::new([self.x(), self.y(), 1.])
    }
}

impl Vec3 {
    #[inline]
    pub fn xy(&self) -> Vec2 {
        Vector([self.0[0], self.0[1]])
    }
    #[inline]
    pub fn x(&self) -> f32 {
        self.0[0]
    }
    #[inline]
    pub fn y(&self) -> f32 {
        self.0[1]
    }
    #[inline]
    pub fn z(&self) -> f32 {
        self.0[2]
    }
    pub fn cross(&self, o: &Self) -> Self {
        let &Vector([x, y, z]) = self;
        let &Vector([a, b, c]) = o;
        Self([y * c - z * b, z * a - x * c, x * b - y * a])
    }
}

#[inline]
fn eq_up_to(v: f32, o: f32, eps: f32) -> bool {
    (v - o).abs() < eps * v.abs().min(o.abs()).min(1.0)
}

#[inline]
pub fn triangle2d_area([p0, p1, p2]: [Vec2; 3]) -> f32 {
    let e0 = p0 - p2;
    let e1 = p1 - p2;
    (e0.x() * e1.y() - e0.y() * e1.x()) / 2.0
}

#[inline]
pub fn triangle3d_area([p0, p1, p2]: [Vec3; 3]) -> f32 {
    (p1 - p0).cross(&(p2 - p0)).length() / 2.0
}

pub fn lines_intersect([a0, a1]: [Vec2; 2], [b0, b1]: [Vec2; 2], eps: f32) -> bool {
    let e0 = a1 - a0;
    let e1 = b1 - b0;
    let denom = -e1.x() * e0.y() + e0.x() * e1.y();
    if eq_up_to(denom, 0., eps) {
        // parallel lines
        return false;
    }
    let s = -e0.y() * (a1.x() - b1.x()) + e0.x() * (a1.y() - b1.y());
    let s = s / denom;
    if s < eps || s > (1. - eps) {
        return false;
    }
    let t = e1.x() * (a1.y() - b1.y()) - e1.y() * (a1.x() - b1.x());
    t > eps && t < 1. - eps
}

macro_rules! impl_ops {
  ($op: ty, $scalar_op: ty, $fn_name: ident, $op_token: tt) => {
    impl<const N: usize> $op for Vector<N> {
      type Output = Self;
      fn $fn_name(self, o: Self) -> Self {
        Self(from_fn(|i| self.0[i] $op_token o.0[i]))
      }
    }
    impl<const N: usize> $scalar_op for Vector<N> {
      type Output = Self;
      fn $fn_name(self, o: f32) -> Self {
        Self(from_fn(|i| self.0[i] $op_token o))
      }
    }
  }
}

impl_ops!(std::ops::Add, std::ops::Add<f32>, add, +);
impl_ops!(std::ops::Sub, std::ops::Sub<f32>, sub, -);
impl_ops!(std::ops::Mul, std::ops::Mul<f32>, mul, *);
impl_ops!(std::ops::Div, std::ops::Div<f32>, div, /);
impl<const N: usize> std::ops::Neg for Vector<N> {
    type Output = Self;
    fn neg(self) -> Self {
        Self(from_fn(|i| -self.0[i]))
    }
}
impl Default for Vec3 {
    fn default() -> Self {
        Vector([0.; 3])
    }
}

macro_rules! impl_assign_ops {
  ($op: ty, $scalar_op: ty, $fn_name: ident, $op_token: tt) => {
    impl<const N: usize> $op for Vector<N> {
      fn $fn_name(&mut self, o: Self) {
        for i in 0..N {
          self.0[i] $op_token o.0[i];
        }
      }
    }

    impl<const N: usize> $scalar_op for Vector<N> {
      fn $fn_name(&mut self, o: f32) {
        for i in 0..N {
          self.0[i] $op_token o;
        }
      }
    }
  }
}

impl_assign_ops!(std::ops::AddAssign, std::ops::AddAssign<f32>, add_assign, +=);
impl_assign_ops!(std::ops::SubAssign, std::ops::SubAssign<f32>, sub_assign, -=);
impl_assign_ops!(std::ops::MulAssign, std::ops::MulAssign<f32>, mul_assign, *=);
impl_assign_ops!(std::ops::DivAssign, std::ops::DivAssign<f32>, div_assign, /=);

#[derive(Debug, PartialEq, Copy, Clone)]
pub struct Extent<const N: usize> {
    min: Vector<N>,
    max: Vector<N>,
}
pub type Extent2 = Extent<2>;
pub type AABB = Extent<3>;

impl<const N: usize> Extent<N> {
    pub fn new(a: Vector<N>, b: Vector<N>) -> Self {
        Self {
            min: a.min(&b),
            max: a.max(&b),
        }
    }
    /// Returns an AABB which contains nothing.
    pub fn empty() -> Self {
        Self {
            min: Vector([f32::INFINITY; N]),
            max: Vector([f32::NEG_INFINITY; N]),
        }
    }
    pub fn reset(&mut self) {
        self.min = Vector([f32::INFINITY; N]);
        self.max = Vector([f32::NEG_INFINITY; N]);
    }
    pub fn add_point(&mut self, p: &Vector<N>) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }
    pub fn add_extent(&mut self, v: &Self) {
        self.min = self.min.min(&v.min);
        self.max = self.max.max(&v.max);
    }
    pub fn expand_by(&mut self, radius: f32) {
        self.min -= radius;
        self.max += radius;
    }
    /// Computes the diagonal of this aabb, which spans the box.
    #[inline]
    pub fn extent(&self) -> Vector<N> {
        self.max - self.min
    }
    /// Returns the center of this extent
    pub fn midpoint(&self) -> Vector<N> {
        self.min + (self.max - self.min) / 2.
    }
    pub fn intersects(&self, other: &Self) -> bool {
        for i in 0..N {
            if self.min.0[i] > other.max.0[i] || self.max.0[i] < other.min.0[i] {
                return false;
            }
        }
        true
    }
    /// Which dimension of this box is largest?
    #[inline]
    pub fn largest_dimension(&self) -> usize {
        self.extent()
            .0
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0
    }
    /// Returns the amount to shift this aabb such that it fits within [-1,1].
    pub fn to_unit(&self) -> (Vector<N>, f32) {
        let center = self.midpoint();
        let scale = self.extent().0[self.largest_dimension()];
        (-center, scale / 2.)
    }
}

/// Computes a tangent to a normal vector.
fn compute_tangent_to(n: &Vec3) -> Vec3 {
    // assume n is unit vector here
    let tangent = if n.x().abs() < n.y().abs() && n.x().abs() < n.z().abs() {
        Vector([1., 0., 0.])
    } else if n.y().abs() < n.z().abs() {
        Vector([0., 1., 0.])
    } else {
        Vector([0., 0., 1.])
    };
    let tangent = tangent - *n * n.dot(&tangent);
    tangent.normalize()
}
fn compute_bitangent_to(n: &Vec3, tangent: &Vec3) -> Vec3 {
    n.cross(tangent)
}

pub mod plane;

#[derive(Debug, Clone, PartialEq)]
struct Basis {
    normal: Vec3,
    tangent: Vec3,
    bitangent: Vec3,
}

#[inline]
pub fn centroid(pts: &[Vec3]) -> Vec3 {
    pts.iter().fold(Vector([0., 0., 0.]), |a, &b| a + b) / (pts.len() as f32)
}

fn covariance(pts: &[Vec3]) -> [f32; 6] {
    let centroid = centroid(pts);
    // TODO is it cleaner to have this as a fold or a for loop
    pts.iter().fold([0.; 6], |mut cov, &p| {
        let v = p - centroid;
        cov[0] += v.x() * v.x();
        cov[1] += v.x() * v.y();
        cov[2] += v.x() * v.z();
        cov[3] += v.y() * v.y();
        cov[4] += v.y() * v.z();
        cov[5] += v.z() * v.z();
        cov
    })
}

impl Basis {
    fn from_eigen(pts: &[Vec3]) -> Option<Basis> {
        let cov = covariance(pts);
        if cov[0] == 0. && cov[3] == 0. && cov[5] == 0. {
            return None;
        }
        let (eigenvalues, eigenvectors) = eigen_solve(cov)?;
        let [t, bit, n] = eigenvectors;
        Some(Self {
            normal: n.normalize(),
            tangent: t.normalize(),
            bitangent: bit.normalize(),
        })
    }
}

/// Computs eigenvalues and eigenvectors for given covariance matrix.
fn eigen_solve(cov: [f32; 6]) -> Option<(Vec3, [Vec3; 3])> {
    let work = [
        [cov[0], cov[1], cov[2]],
        [cov[1], cov[3], cov[4]],
        [cov[2], cov[4], cov[5]],
    ];
    let (mat, diag, subd) = householder_transformation(&work);
    todo!();
}

// takes a symmetric matrix, and performs a householder transformation to get
// orthogonal matrix Q, diagonal t, and subdiagonal elements.
fn householder_transformation(mat: &[[f32; 3]; 3]) -> ([[f32; 3]; 3], Vec3, [f32; 3]) {
    const EPS: f32 = 1e-8;
    let &[[a, b, c], [_, d, e], [_, _, f]] = mat;
    if c.abs() >= EPS {
        let ell = (b * b + c * c).sqrt();
        let b = b / ell;
        let c = c / ell;
        let q = 2. * b * e + c * (f - d);
        (
            [[1., 0., 0.], [0., b, c], [0., c, -b]],
            Vector([a, d + c * q, f - c * q]),
            [ell, e - b * q, 0.],
        )
    } else {
        (
            [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            Vector([a, d, f]),
            [b, e, 0.],
        )
    }
}

// QL iteration with implicit shifting to reduce matrix from tridiagonal
// to diagonal
fn qla_algorithm(mut q: [[f32; 3]; 3], mut diag: Vec3, mut subd: [f32; 3]) {
    const MAX_ITER: usize = 32;
    for dim in 0..3 {
        for iter in 0..MAX_ITER {
            assert_ne!(iter, MAX_ITER);
            let mut m = dim;
            while m <= 1 {
                // let dd = diag.0[m].abs() + diag.0[m+1].abs()
                // This line differs from original, but I'm not sure why the origina was like it
                // was.
                // https://github.com/jpcy/xatlas/blob/16ace528acd2cf1f16a7c0dde99c42c486488dbe/source/xatlas/xatlas.cpp#L1858
                if subd[m].abs() == 0. {
                    break;
                }
                m += 1
            }
            if m == dim {
                break;
            }
            let g = (diag.0[dim + 1] - diag.0[dim]) / (2.0 * subd[dim]);
            let mut r = (g * g + 1.).sqrt();
            let mut g = diag.0[m] - diag.0[dim] + subd[dim] / (g + (if g < 0. { -r } else { r }));

            let mut s = 1.;
            let mut c = 1.;
            let mut p = 0.;
            for i in (dim..=(m - 1)).rev() {
                let mut f = s * subd[i];
                let b = c * subd[i];
                if f.abs() >= g.abs() {
                    c = g / f;
                    r = (c * c + 1.).sqrt();
                    s = 1. / r;
                    c *= s;
                } else {
                    s = f / g;
                    r = (s * s + 1.).sqrt();
                    subd[i + 1] = g * r;
                    c = 1. / r;
                    s *= c;
                }
                g = diag.0[i + 1] - p;
                r = (diag.0[i] - g) * s + 2. * b * c;
                p = s * r;
                diag.0[i + 1] = g + p;
                g = c * r - b;
                for k in 0..3 {
                    f = q[k][i + 1];
                    q[k][i + 1] = s * q[k][i] + c * f;
                    q[k][i] = c * q[k][i] - s * f;
                }
            }
            diag.0[dim] -= p;
            subd[dim] = g;
            subd[m] = 0.;
        }
    }
}

pub mod bb2d;
use bb2d::{turn_kind, TurnKind};

/// Checks if a 2D point is within a triangle
fn point_in_triangle(p: &Vec2, [a, b, c]: &[Vec2; 3]) -> bool {
    [turn_kind(p, a, b), turn_kind(p, b, c), turn_kind(p, c, a)]
        .into_iter()
        .all(|s| s == TurnKind::Left || s == TurnKind::Collinear)
}

/// Creates a mesh grid of points, i.e. evenly spaced between min and max, with a given width
/// and height
fn mesh_grid<const X: usize, const Y: usize>(min: Vec2, max: Vec2) -> [[Vec2; X]; Y] {
    let bd = max - min;
    std::array::from_fn(|y| {
        std::array::from_fn(|x| {
            let dx = (x as f32 / X as f32);
            let dy = (y as f32 / Y as f32);
            min + bd * Vector([dx, dy])
        })
    })
}

#[test]
fn test_point_in_triangle() {
    let mut pv = point_vis::PointVisualizer::new();
    let pts = mesh_grid::<100, 100>(Vector([0.25; 2]), Vector([0.75; 2]));
    let triangle = [Vector([0.3, 0.3]), Vector([0.7, 0.3]), Vector([0.5, 0.7])];
    for row in pts {
        for p in row {
            pv.add_point(
                p,
                if point_in_triangle(&p, &triangle) {
                    [0, 255, 0]
                } else {
                    [255, 0, 0]
                },
            );
        }
    }
    for p in triangle {
        pv.add_point(p, [0, 0, 255]);
    }
    pv.save("test_point_in_triangle.png");
}

#[derive(Debug, PartialEq, Clone)]
pub struct RasterTri {
    verts: [Vec2; 3],
    normals: [Vec2; 3],
}

impl RasterTri {
    fn new(mut verts: [Vec2; 3]) -> Self {
        Self::ensure_forward_facing(&mut verts);
        let normals = if triangle2d_area(verts) > 0. {
            Self::compute_inward_normals(&verts)
        } else {
            [Vector::zero(); 3]
        };
        Self { verts, normals }
    }
    fn ensure_forward_facing(verts: &mut [Vec2; 3]) {
        if turn_kind(&verts[0], &verts[1], &verts[2]) == TurnKind::Right {
            verts.swap(0, 1);
            assert_eq!(turn_kind(&verts[0], &verts[1], &verts[2]), TurnKind::Left)
        }
    }
    fn compute_inward_normals(&[v0, v1, v2]: &[Vec2; 3]) -> [Vec2; 3] {
        [
            (v0 - v1).ortho().normalize(),
            (v1 - v2).ortho().normalize(),
            (v2 - v0).ortho().normalize(),
        ]
    }
}

pub mod mesh;

pub fn point_on_line(&p: &Vec3, &[l0, l1]: &[Vec3; 2], eps: f32) -> Option<f32> {
    if p.approx_equals(&l0, eps) {
        return Some(0.);
    } else if p.approx_equals(&l1, eps) {
        return Some(1.);
    }
    if triangle3d_area([p, l0, l1]) > eps {
        return None;
    }
    let line = l1 - l0;
    let t = (p - l0).dot(&line) / line.length();
    Some(t)
}

/// An intersection of a ray with the surface of mesh.
/// Contains the distance along the ray, and face index it intersected.
#[derive(Debug, Clone, Copy)]
pub struct Intersection {
    pub face: usize,
    pub t: f32,
}

/// Surfaces are objects which can be hit by rays.
pub trait Surface {
    fn intersect_ray(&self, ray: &Ray) -> Option<Intersection>;
}

pub mod obj;

/// A ray with an origin, direction, and a length
#[derive(Debug, Clone, PartialEq)]
pub struct Ray {
    pub origin: Vec3,
    pub dir: Vec3,
}

impl Ray {
    /// Returns the position `t` distance along this ray.
    #[inline]
    pub fn at(&self, t: f32) -> Vec3 {
        self.origin + self.dir * t
    }
}

pub fn intersect_tri(&[p0, p1, p2]: &[Vec3; 3], r: &Ray, eps: f32) -> Option<f32> {
    let e0 = p1 - p0;
    let e1 = p2 - p0;
    let h = r.dir.cross(&e1);
    let a = e0.dot(&h);
    if -eps < a && a < eps {
        return None;
    }
    let f = 1. / a;
    let s = r.origin - p0;
    let u = f * s.dot(&h);
    if u < 0. || u > 1. {
        return None;
    }
    let q = s.cross(&e0);
    let v = f * r.dir.dot(&q);
    if v < 0. || u + v > 1. {
        return None;
    }
    let t = f * e1.dot(&q);
    Some(t).filter(|t| *t > eps)
}
