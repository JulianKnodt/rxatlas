#![allow(non_snake_case)]

use super::{Edge, Mesh};
use crate::rand::rand_vec;
use crate::{Vec2, Vec3, Vector};

impl Mesh {
    /// Extremely shitty way to get a tutte parameterization.
    /// Instead of solving an actual system of equations,
    /// just iterate until it stops updating within some threshold.
    pub fn tutte_parameterization(&self) -> Vec<Vec2> {
        let V = self.verts.len();
        // boundary verts are fixed, everything else is an average;
        let mut to_solve = vec![true; V];
        let mut inits = vec![rand_vec::<2>().clamp_normalize(); V];
        for (bi, pos) in self.map_boundary_verts_to_circle() {
            inits[bi] = pos;
            to_solve[bi] = false;
        }
        let mut next = inits.clone();
        let adj_counts = self.adjacency_counts();
        assert!(adj_counts.iter().all(|&adj| adj > 0));

        let mut prev_change = 1e5;
        let mut total_change = 0.0;
        loop {
            let mut next = inits.clone();
            for (i, n) in next.iter_mut().enumerate() {
                if to_solve[i] {
                    *n = Vec2::zero();
                }
            }

            for f in self.faces() {
                for Edge([vi0, vi1]) in f.edges() {
                    if to_solve[vi0] {
                        let delta = inits[vi1] / adj_counts[vi0] as f32;
                        total_change += delta.length_sq();
                        next[vi0] += delta;
                    }
                    if to_solve[vi1] {
                        let delta = inits[vi0] / adj_counts[vi1] as f32;
                        total_change += delta.length_sq();
                        next[vi1] += delta;
                    }
                }
            }
            std::mem::swap(&mut inits, &mut next);
            assert!(total_change.is_normal());
            if (prev_change - total_change).abs() < 1e-3 {
                break;
            }
            prev_change = total_change;
            total_change = 0.0;
        }

        // create equations with equivalent weight on each adjacent vertex (1/2)
        // Then solve the system of equations.
        inits
    }
}
