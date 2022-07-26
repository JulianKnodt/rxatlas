use super::{Vec2, Vec3, Vector};

#[inline]
/// Shitty randomizer for values in [0,1]
pub fn rand(seed: f32) -> f32 {
    static mut C: f32 = 1.0;
    unsafe {
        let v = (C * 4997.39 + 3.8).sin();
        let v = (v + 1.) / 2.;
        C += v + seed;
        v
    }
}

#[inline]
/// Returns a random vector with all elements in [0,1].
pub fn rand_vec<const N: usize>() -> Vector<N> {
    Vector(std::array::from_fn(|i| rand(i as f32)))
}

#[inline]
/// Returns a random position within the unit circle [-1, 1]
pub fn rand_unit_circle(seed: f32) -> Vec2 {
    let l = rand(seed).sqrt();
    let theta = 2.0 * std::f32::consts::PI * rand(seed + 1.);
    let (s, c) = theta.sin_cos();
    Vec2::new([l * c, l * s])
}

#[inline]
/// Returns an infinite iterator over quasi-random values in [0,1].
/// https://en.wikipedia.org/wiki/Low-discrepancy_sequence#Random_numbers
pub fn quasi_random_iter() -> impl Iterator<Item = f32> {
    (0..).map(|i| {
        let v = rand(i as f32) * 0.5;
        if i % 2 == 0 {
            v + 0.5
        } else {
            v
        }
    })
}

// https://alexanderameye.github.io/notes/sampling-the-hemisphere/

/// Returns a random uniform sample in the hemisphere around ([0,0,1])
pub fn uniform_hemisphere(seed: f32) -> Vec3 {
    let theta = rand(seed).acos();
    let phi = 2.0 * std::f32::consts::PI * rand(seed);
    Vector([theta, phi]).elaz_to_xyz()
}

#[test]
fn test_uniform_hemisphere() {
    for i in 0..100 {
        let v = uniform_hemisphere(i as f32);
        assert!(v.z() >= 0.);
    }
}
