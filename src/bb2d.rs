use super::{Vec2, Vector};
use std::cmp::Ordering;

// TODO what was this for?
/// Not necessarily aligned bounding box
#[derive(Debug)]
pub struct BoundingBox2D {
    major_axis: Vec2,
    minor_axis: Vec2,
    min_corner: Vec2,
    max_corner: Vec2,

    boundary_verts: Vec<Vec2>,
    coords: Vec<f32>,

    top: Vec<Vec2>,
    bot: Vec<Vec2>,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum TurnKind {
    Collinear,
    Right,
    Left,
}

// indicates whether a --> b --> c is a left turn or not
pub fn turn_kind(a: &Vec2, b: &Vec2, c: &Vec2) -> TurnKind {
    match (*b - *a).cross(&(*c - *a)).partial_cmp(&0.).unwrap() {
        Ordering::Equal => TurnKind::Collinear,
        Ordering::Greater => TurnKind::Left,
        Ordering::Less => TurnKind::Right,
    }
}

fn convex_hull(pts: &[Vec2]) -> Vec<Vec2> {
    if pts.is_empty() {
        return vec![];
    }

    let p = pts
        .iter()
        .min_by(|a, b| {
            a.y()
                .total_cmp(&b.y())
                .then_with(|| a.x().total_cmp(&b.x()))
        })
        .unwrap();

    let mut pt_clone = pts.to_vec();
    pt_clone.sort_unstable_by(|&a, &b| {
        let e0 = (a - *p).normalize().x();
        let e1 = (b - *p).normalize().x();
        e0.total_cmp(&e1)
    });

    let mut out = vec![pt_clone[0], pt_clone[1]];

    for (i, pt) in pt_clone.iter().enumerate().skip(2) {
        while out.len() >= 2
            && turn_kind(&out[out.len() - 2], &out[out.len() - 1], pt) == TurnKind::Left
        {
            assert!(out.pop().is_some());
        }
        out.push(*pt);
    }
    out
}

#[test]
fn test_convex_hull() {
    use super::rand::rand_vec;

    let mut pv = super::point_vis::PointVisualizer::new();
    //let pts: [_; 100] = std::array::from_fn(|_| (rand_vec::<2>() - 0.5).normalize() * 0.5 + 0.5);
    let pts: [_; 100] = std::array::from_fn(|_| rand_vec::<2>() * 0.5 + 0.25);
    for p in pts {
        pv.add_point(p, [255, 0, 0]);
    }
    // reference point
    pv.add_point(Vector([0.1, 0.1]), [255, 255, 0]);

    for p in convex_hull(&pts) {
        pv.add_point(p, [0, 255, 0]);
    }
    pv.save("pv_test.png");
}
