#![feature(let_else)]
#![allow(unused)]
use std::array::from_fn;
use std::cmp::Ordering;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

mod bvh;
pub mod point_vis;
pub mod rand;
pub mod triangle;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Vector<const N: usize, T = f32>(pub [T; N]);
pub type Vec2<T = f32> = Vector<2, T>;
pub type Vec3<T = f32> = Vector<3, T>;
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
    pub fn clamp(&self, min: f32, max: f32) -> Self {
        Self(from_fn(|i| self.0[i].min(max).max(min)))
    }
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.0.iter().copied().all(f32::is_finite)
    }
    #[inline]
    pub fn abs(&self) -> Self {
        Self(self.0.map(|v| v.abs()))
    }
    #[inline]
    pub fn floor(&self) -> Self {
        Self(self.0.map(f32::floor))
    }
    #[inline]
    pub fn ceil(&self) -> Self {
        Self(self.0.map(f32::ceil))
    }
    pub fn cast<T>(&self) -> [T; N]
    where
        f32: Into<T>,
    {
        self.0.map(|v| v.into())
    }
}

impl<const N: usize, T: PartialOrd + Copy> Vector<N, T> {
    pub fn min(&self, o: &Self) -> Self {
        Self(from_fn(|dim| match self.0[dim].partial_cmp(&o.0[dim]) {
            None | Some(Ordering::Less) | Some(Ordering::Equal) => self.0[dim],
            Some(Ordering::Greater) => o.0[dim],
        }))
    }
    pub fn max(&self, o: &Self) -> Self {
        Self(from_fn(|dim| match self.0[dim].partial_cmp(&o.0[dim]) {
            None | Some(Ordering::Greater) | Some(Ordering::Equal) => self.0[dim],
            Some(Ordering::Less) => o.0[dim],
        }))
    }
}

impl<T: Copy> Vec2<T> {
    #[inline]
    pub fn x(&self) -> T {
        self.0[0]
    }
    #[inline]
    pub fn y(&self) -> T {
        self.0[1]
    }
    #[inline]
    pub fn u(&self) -> T {
        self.0[0]
    }
    #[inline]
    pub fn v(&self) -> T {
        self.0[1]
    }
}

impl Vec2 {
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
    /// Find where the line Ax + b intersects a given coordinate along some dimension
    /// If parallel, may return NaN.
    pub fn line_intersects(a: Vec2, b: Vec2, dim: usize, val: f32) -> f32 {
        assert!(dim <= 1, "Only two valid dimensions");
        (val - b.0[dim]) / a.0[dim]
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

    #[inline]
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
  ($op: ty, $gen_ty: ident, $scalar_op: ty, $fn_name: ident, $op_token: tt) => {
    impl<const N: usize> $op for Vector<N> {
      type Output = Self;
      fn $fn_name(self, o: Self) -> Self {
        Self(from_fn(|i| self.0[i] $op_token o.0[i]))
      }
    }
    impl<const N: usize, $gen_ty: PartialOrd + Copy +
      Add<Output=T> + Mul<Output=T> + Sub<Output=T> + Div<Output=T>> $scalar_op for Vector<N, T> {
      type Output = Self;
      fn $fn_name(self, o: $gen_ty) -> Self {
        Self(from_fn(|i| self.0[i] $op_token o))
      }
    }
  }
}

impl_ops!(Add, T, Add<T>, add, +);
impl_ops!(Sub, T, Sub<T>, sub, -);
impl_ops!(Mul, T, Mul<T>, mul, *);
impl_ops!(Div, T, Div<T>, div, /);

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
  ($op: ty, $gen_ty: ident, $scalar_op: ty, $fn_name: ident, $op_token: tt) => {
    impl<const N: usize> $op for Vector<N> {
      fn $fn_name(&mut self, o: Self) {
        for i in 0..N {
          self.0[i] $op_token o.0[i];
        }
      }
    }

    impl<const N: usize, $gen_ty> $scalar_op for Vector<N, T> where
      $gen_ty: PartialOrd + Copy + AddAssign + SubAssign + MulAssign + DivAssign {
      fn $fn_name(&mut self, o: T) {
        for i in 0..N {
          self.0[i] $op_token o;
        }
      }
    }
  }
}

impl_assign_ops!(AddAssign, T, AddAssign<T>, add_assign, +=);
impl_assign_ops!(SubAssign, T, SubAssign<T>, sub_assign, -=);
impl_assign_ops!(MulAssign, T, MulAssign<T>, mul_assign, *=);
impl_assign_ops!(DivAssign, T, DivAssign<T>, div_assign, /=);

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub struct Extent<const N: usize, T = f32> {
    pub min: Vector<N, T>,
    pub max: Vector<N, T>,
}

pub type AABB2<T = f32> = Extent<2, T>;

pub type AABB<T = f32> = Extent<3, T>;

impl AABB2<u32> {
    #[inline]
    pub fn iter_bounds(&self) -> impl Iterator<Item = [u32; 2]> {
        let [lx, ly] = self.min.0;
        let [hx, hy] = self.max.0;
        (ly..hy).flat_map(move |y| (lx..hx).map(move |x| [x, y]))
    }
    #[inline]
    pub fn unit_squares(self) -> impl Iterator<Item = Self> {
        (self.min.x()..self.max.x()).flat_map(move |x| {
            (self.min.y()..self.max.y()).map(move |y| {
                let p = Vector::new([x, y]);
                Extent::new(p, p + 1)
            })
        })
    }
}

impl AABB2 {
    #[inline]
    pub fn intersects_tri2<T>(&self, t: &triangle::Triangle2<T>) -> bool {
        let [v0, v1, v2] = t.verts;
        // Check if the aabb contains any of the triangle's vertices
        let tri_in_box =
            self.contains_point(&v0) || self.contains_point(&v1) || self.contains_point(&v2);
        if tri_in_box {
            return true;
        }

        // Check if the box contains any corner of the triangle triangle
        let box_in_tri = self.corners().into_iter().any(|c| t.contains(c));
        if box_in_tri {
            return true;
        }
        // otherwise, look at equation of each edge, compute where it would intersect the bounding
        // box, and see if it in the bounds
        let edge_intersects = move |a, b| {
            let edge = a - b;
            let t0 = Vec2::line_intersects(edge, b, 0, self.min.x());
            let t1 = Vec2::line_intersects(edge, b, 0, self.max.x());

            let y0 = b.y() + edge.y() * t0;
            let y1 = b.y() + edge.y() * t1;

            let y_min = y0.min(y1);
            let y_max = y0.max(y1);

            // if contains either y
            (y_min <= self.min.y() && self.min.y() <= y_max)
                || (y_min <= self.max.y() && self.max.y() <= y_min)
        };
        edge_intersects(v1, v0) || edge_intersects(v2, v1) || edge_intersects(v2, v0)
    }
}

impl<T: PartialOrd + Copy> AABB2<T> {
    #[inline]
    pub fn corners(&self) -> [Vec2<T>; 4] {
        [
            self.min,
            Vector::new([self.min.x(), self.max.y()]),
            Vector::new([self.max.x(), self.min.y()]),
            self.max,
        ]
    }
}

impl<const N: usize, T: PartialOrd + Copy> Extent<N, T> {
    #[inline]
    pub fn new(a: Vector<N, T>, b: Vector<N, T>) -> Self {
        Self {
            min: a.min(&b),
            max: a.max(&b),
        }
    }
    /// Returns if a value is within a dimension
    #[inline]
    fn within_dim(&self, dim: usize, v: T) -> bool {
        self.min.0[dim] <= v && v <= self.max.0[dim]
    }
    #[inline]
    pub fn contains_point(&self, p: &Vector<N, T>) -> bool {
        p.0.iter()
            .enumerate()
            .all(|(dim, &c)| self.within_dim(dim, c))
    }
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.min.0.iter().zip(self.max.0).any(|(&l, h)| l >= h)
    }
}

impl<const N: usize, T: Copy> Extent<N, T>
where
    Vector<N, T>: Add<T, Output = Vector<N, T>> + Sub<T, Output = Vector<N, T>>,
{
    pub fn expand_by(&self, radius: T) -> Self {
        Self {
            min: self.min - radius,
            max: self.max + radius,
        }
    }
}

impl<const N: usize> Extent<N> {
    /// Returns an AABB which contains nothing.
    #[inline]
    pub fn empty() -> Self {
        Self {
            min: Vector([f32::INFINITY; N]),
            max: Vector([f32::NEG_INFINITY; N]),
        }
    }
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::empty();
    }
    #[inline]
    pub fn add_point(&mut self, p: &Vector<N>) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }
    pub fn add_extent(&mut self, v: &Self) {
        self.min = self.min.min(&v.min);
        self.max = self.max.max(&v.max);
    }
    /// Computes the diagonal of this aabb, which spans the box.
    #[inline]
    pub fn extent(&self) -> Vector<N> {
        self.max - self.min
    }
    /// Returns the center of this extent
    #[inline]
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
    /// Returns the largest dimension of this AABB, as well as its value.
    #[inline]
    pub fn largest_dimension(&self) -> (usize, f32) {
        self.extent()
            .0
            .iter()
            .copied()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
    }
    /// Returns the amount to shift this aabb such that it fits within [-1,1].
    #[inline]
    pub fn to_unit(&self) -> (Vector<N>, f32) {
        let center = self.midpoint();
        let scale = self.largest_dimension().1;
        (-center, scale / 2.)
    }

    /// Returns the shift and scale needed to convert this aabb into the other.
    pub fn to_other(&self, o: &Self) -> (Vector<N>, Vector<N>) {
        (o.midpoint() - self.midpoint(), o.extent() / self.extent())
    }
    #[inline]
    pub fn to_u32(&self) -> Extent<N, u32> {
        Extent {
            min: Vector::new(self.min.floor().0.map(|v| v as u32)),
            max: Vector::new(self.max.ceil().0.map(|v| v as u32)),
        }
    }
}

impl<const N: usize> Extent<N, u32> {
    #[inline]
    pub fn to_f32(&self) -> Extent<N, f32> {
        Extent {
            min: Vector::new(self.min.0.map(|v| v as f32)),
            max: Vector::new(self.max.0.map(|v| v as f32)),
        }
    }
}

impl<const N: usize> Mul<f32> for Extent<N> {
    type Output = Self;
    fn mul(self, o: f32) -> Self {
        Self {
            min: self.min * o,
            max: self.max * o,
        }
    }
}

impl<const N: usize> Mul<Vector<N>> for Extent<N> {
    type Output = Self;
    fn mul(self, o: Vector<N>) -> Self {
        Self {
            min: self.min * o,
            max: self.max * o,
        }
    }
}

impl<const N: usize> Div<Vector<N>> for Extent<N> {
    type Output = Self;
    fn div(self, o: Vector<N>) -> Self {
        Self {
            min: self.min / o,
            max: self.max / o,
        }
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
                for k in &mut q {
                    f = k[i + 1];
                    k[i + 1] = s * k[i] + c * f;
                    k[i] = c * k[i] - s * f;
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
    #[inline]
    pub fn new(origin: Vec3, dir: Vec3) -> Self {
        Ray { origin, dir }
    }
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
