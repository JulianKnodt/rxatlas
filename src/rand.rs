use super::Vector;

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

/// Returns a random vector with all elements in [0,1].
pub fn rand_vec<const N: usize>() -> Vector<N> {
    Vector(std::array::from_fn(|i| rand(i as f32)))
}
