use super::{Basis, Vec3, Vector};

/// Represents a plane in 3D.
#[derive(Debug)]
pub struct Plane {
    point: Vec3,
    normal: Vec3,
}

impl Plane {
    fn basis(&self) -> Basis {
        let normal = &self.normal;
        let tangent = super::compute_tangent_to(normal);
        let bitangent = super::compute_bitangent_to(normal, &tangent);
        Basis {
            normal: *normal,
            tangent,
            bitangent,
        }
    }
}

/// Constructs a plane from a series of points based on
/// https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
/// https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
fn plane_from_points(pts: &[Vec3]) -> Option<Plane> {
    assert!(pts.len() >= 3);
    if let &[p0, p1, p2] = pts {
        let center = (p0 + p1 + p2) / 3.0;
        return Some(Plane {
            normal: (p2 - p0).cross(&(p1 - p0)),
            point: center,
        });
    }
    let mut centroid: Vec3 = Vector::zero();
    for p in pts {
        centroid += *p;
    }
    let n = pts.len() as f32;
    centroid /= n;

    let mut xx = 0.;
    let mut xy = 0.;
    let mut xz = 0.;
    let mut yy = 0.;
    let mut yz = 0.;
    let mut zz = 0.;
    for p in pts {
        let r = *p - centroid;
        xx += r.x() * r.x();
        xy += r.x() * r.y();
        xz += r.x() * r.z();
        yy += r.y() * r.y();
        yz += r.y() * r.z();
        zz += r.z() * r.z();
    }
    xx /= n;
    xy /= n;
    xz /= n;
    yy /= n;
    yz /= n;
    zz /= n;

    let mut weighted_dir = Vector::zero();

    {
        let det_x = yy * zz - yz * yz;
        let axis_dir = Vector([det_x, xz * yz - xy * zz, xy * yz - xz * yy]);
        let mut weight = det_x * det_x;
        if weighted_dir.dot(&axis_dir) < 0. {
            weight = -weight
        }
        weighted_dir += axis_dir * weight;
    }

    {
        let det_y = xx * zz - xz * xz;
        let axis_dir = Vector([xz * yz - xy * zz, det_y, xy * xz - yz * xx]);
        let mut weight = det_y * det_y;
        if weighted_dir.dot(&axis_dir) < 0. {
            weight = -weight
        }
        weighted_dir += axis_dir * weight;
    }

    {
        let det_z = xx * yy - xy * xy;
        let axis_dir = Vector([xy * yz - xz * yy, xy * xz - yz * xx, det_z]);
        let mut weight = det_z * det_z;
        if weighted_dir.dot(&axis_dir) < 0. {
            weight = -weight
        }
        weighted_dir += axis_dir * weight;
    }
    let normal = weighted_dir.normalize();
    Some(Plane {
        point: centroid,
        normal,
    })
    .filter(|_| normal.is_finite())
}
