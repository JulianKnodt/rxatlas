use super::Mesh;
use crate::triangle::Triangle3;
use crate::{rand, Vec3};

impl Mesh {
    /// Returns a map from face idx to some number of groups.
    pub fn group_faces(&self, clusters: usize) -> Vec<usize> {
        #[allow(non_snake_case)]
        let K = clusters;
        const UNSET: usize = usize::MAX;
        let mut groups = vec![UNSET; self.num_faces()];
        let mut curr_costs = vec![f32::INFINITY; self.faces.len()];
        const LAMBDA: f32 = 1.;

        // In order to pick initial seed, pick face, grow it, then next seed is worst on that
        // chart
        let mut chart_sizes = vec![1; K];
        // keep a list of normal + centroid for each chart
        let mut chart_summary = vec![(Vec3::zero(), Vec3::zero()); K];

        let target_num = self.faces.len() / clusters;
        assert_ne!(target_num, 0);
        assert_ne!(target_num, 1, "Just assigns 1 face to each cluster");
        let tri_for_face = |i| self.face_tri(i).map(|t, ()| (t.normal(), t.centroid()));

        // TODO probably want to make a generic function over costs.
        macro_rules! cost {
            ($tri: expr, $chart_num: expr) => {{
                let (chart_normal, chart_centroid) = chart_summary[$chart_num];
                let (tri_normal, tri_centroid) = $tri.data;
                let dist: Vec3 = chart_centroid - tri_centroid;
                (LAMBDA - tri_normal.dot(&chart_normal)) * dist.length()
            }};
        }

        macro_rules! update_chart_summary {
            ($tri: expr, $chart_num: expr) => {{
                let (chart_normal, chart_centroid) = chart_summary[$chart_num];
                let (tri_normal, tri_centroid) = $tri.data;
                let w = 1. / chart_sizes[$chart_num] as f32;
                chart_summary[$chart_num] = (
                    chart_normal * (1. - w) + tri_normal * w,
                    chart_centroid * (1. - w) + tri_centroid * w,
                );
            }};
        }

        // grow seed will grow from a seed, and eventually returns the seed.
        // Threshold is when the chart is about 1/
        let mut grow_seed = |seed, chart_num| {
            groups[seed] = chart_num;
            let mut chart_edge = vec![seed];

            chart_summary[chart_num] = tri_for_face(seed).data;

            while let Some(work) = chart_edge.pop() {
                for af in self.adjacent_faces(work) {
                    if groups[af] == chart_num {
                        // already in this group, don't need to add it
                        continue;
                    }
                    // otherwise check if this face should be inside this chart
                    let tri = tri_for_face(af);
                    if cost!(tri, chart_num) < curr_costs[af] {
                        groups[af] = chart_num;
                        update_chart_summary!(tri, chart_num);
                        chart_edge.push(af);
                    }
                }
                if chart_sizes[chart_num] >= target_num {
                    break;
                }
            }

            // iterate over chart, find best and worst items
            let Some(next) = chart_edge.pop() else { todo!() };
            let c = cost!(tri_for_face(next), chart_num);
            let (best, best_cost, worst, _) = chart_edge.into_iter().fold(
                (next, c, next, c),
                |(best, bcost, worst, wcost), n| {
                    let n_cost = cost!(tri_for_face(n), chart_num);
                    if n_cost > wcost {
                        (best, bcost, n, n_cost)
                    } else if n_cost < bcost {
                        (n, n_cost, worst, wcost)
                    } else {
                        (best, bcost, worst, wcost)
                    }
                },
            );
            (best, best_cost, worst)
        };

        let mut seeds = vec![(UNSET, f32::INFINITY, Triangle3::default()); K];

        // grow each cluster until it
        let mut curr_seed = 0;
        for i in 0..K {
            let (best, best_cost, next_seed) = grow_seed(curr_seed, i);
            seeds[i] = (best, best_cost, tri_for_face(best));
            curr_seed = next_seed;
        }

        macro_rules! seed_cost {
            ($tri: expr, $chart_num: expr) => {{
                let (chart_normal, chart_centroid) = seeds[$chart_num].2.data;
                let (tri_normal, tri_centroid) = $tri.data;
                let dist: Vec3 = chart_centroid - tri_centroid;
                (LAMBDA - tri_normal.dot(&chart_normal)) * dist.length()
            }};
        }

        let mut did_change = true;
        while did_change {
            did_change = false;
            // assign each face to a chart
            for i in 0..self.num_faces() {
                let f = tri_for_face(i);
                let assn = (0..K)
                    .map(|k| (k, seed_cost!(f, k)))
                    .min_by(|(_, c0), (_, c1)| c0.total_cmp(c1))
                    .unwrap()
                    .0;
                if groups[i] != assn {
                    did_change = true;
                    groups[i] = assn;
                }
            }
            // Pick seeds
            for (i, &assn) in groups.iter().enumerate() {
                assert_ne!(assn, UNSET);

                let tri = tri_for_face(i);
                let c = seed_cost!(tri, assn);
                if c < seeds[assn].1 {
                    seeds[assn] = (i, c, tri);
                }
            }
        }

        groups
    }
}
