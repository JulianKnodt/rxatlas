use super::Vector;
use std::array::from_fn;
use std::collections::BTreeMap;

// TODO add another parameter for col/rows
pub type Matrix<const N: usize> = [Vector<N>; N];
// TODO need to implement some kind of solver, what are the exact input and output

/// Returns an identity matrix.
#[inline]
pub fn eye<const N: usize>() -> Matrix<N> {
    from_fn(|i| {
        let mut row = Vector::zero();
        row.0[i] = 1.;
        row
    })
}

/// Returns an empty matrix
#[inline]
pub fn zeros<const N: usize>() -> Matrix<N> {
    from_fn(|_| Vector::zero())
}

/// Performs simple matrix multiple
#[inline]
pub fn matmul<const N: usize>(a: &Matrix<N>, b: &Matrix<N>) -> Matrix<N> {
    let mut out = zeros();
    for i in 0..N {
        for j in 0..N {
            // TODO I think this can be "vectorized" better?
            for k in 0..N {
                out[i].0[j] += a[i].0[k] * b[k].0[j];
            }
        }
    }
    out
}

/// Performs matrix vector multiplication `Ab`
#[inline]
pub fn vecmul<const N: usize>(a: &Matrix<N>, b: &Vector<N>) -> Vector<N> {
    Vector::new(a.map(|row| row.dot(b)))
}

fn is_lower_triangular<const N: usize>(m: &Matrix<N>) -> bool {
    for y in 0..N {
        for x in y + 1..N {
            if m[y].0[x] != 0. {
                return false;
            }
        }
    }
    true
}

/// Solves an exact system of linear equations `Ax = b`, where A: R^(n,n), b: R^n, x: R^n.
/// If there are no solutions, returns None, and if there are multiple solutions returns an
/// arbitrary choice.
///
/// This assumes floating point representation, and does not work for integers
pub trait ExactSolver {
    fn solve<const N: usize>(&self, a: &Matrix<N>, b: &Vector<N>) -> Option<Vector<N>>;
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct TriangularSolver;

/// An LU Decomposition without pivoting
#[derive(Debug, Clone, PartialEq)]
pub struct LUDecomposition<const N: usize> {
    /// Lower Triangular Matrix
    l: Matrix<N>,
    /// Upper Triangular Matrix
    u: Matrix<N>,
}

// https://www.nicolasboumal.net/papers/MAT321_Lecture_notes_Boumal_2019.pdf

/// Performs an LU decomposition of this matrix, such that L is lower triangular, U is upper
/// triangular, and `LU = mat`.
pub fn lu<const N: usize>(mat: &Matrix<N>) -> Option<LUDecomposition<N>> {
    let mut l = mat.clone();
    let mut u = eye();

    // for each element along the diagonal except the last one
    for k in 0..N - 1 {
        let diag = l[k][k];
        // TODO probably want an approx check here, but maybe exact is fine?
        if diag == 0. {
            return None;
        }
        // for each row below
        for j in k + 1..N {
            u[k][j] = l[k][j] / diag;
            for i in k..N {
                l[i][j] -= l[i][k] * u[k][j];
            }
        }
    }

    Some(LUDecomposition { l, u })
}

pub fn back_substitute_upper<const N: usize>(u: &Matrix<N>, b: &Vector<N>) -> Vector<N> {
    let mut out = Vector::zero();
    for k in (0..N).rev() {
        let diag = u[k][k];
        assert_ne!(diag, 0.);
        let prev = (k + 1..N).map(|j| out[j] * u[k][j]).sum::<f32>();
        out[k] = (b[k] - prev) / diag
    }
    out
}

pub fn back_substitute_lower<const N: usize>(l: &Matrix<N>, b: &Vector<N>) -> Vector<N> {
    let mut out = Vector::zero();
    for k in 0..N {
        let diag = l[k][k];
        assert_ne!(diag, 0.);
        let prev = (0..k).map(|j| out[j] * l[k][j]).sum::<f32>();
        out[k] = (b[k] - prev) / diag
    }
    out
}

#[test]
fn test_simple_lu() {
    let mat = eye::<3>();
    let lud = lu(&mat).expect("Failed to decompose eye matrix");

    let mat = [
        Vector::new([1., 2., 3.]),
        Vector::new([4., 5., 6.]),
        Vector::new([7., 8., 9.5]),
    ];
    let lud = lu(&mat).expect("Failed to decompose eye matrix");
    assert!(is_lower_triangular(&lud.l));
    // TODO this check is dangerous cause of FP impression
    assert_eq!(mat, matmul(&lud.l, &lud.u));

    let x = Vector::new([13., 15., 17.]);
    let b = vecmul(&mat, &x);

    let x_got = back_substitute_upper(&lud.u, &back_substitute_lower(&lud.l, &b));
    println!("{:?}", (x_got, x));
}

/// A sparse matrix is represented as a btree that maps coordinates to floats.
pub type SparseMatrix<T = f32> = BTreeMap<[usize; 2], T>;
