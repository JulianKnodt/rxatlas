use clap::Parser;

/// Texture Baker, which takes a low-poly, UV unwrapped mesh
/// and applies the texture of a high-poly mesh to it.
#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Name of the person to greet
    #[clap(short, long, value_parser)]
    low_poly: String,

    /// Number of times to greet
    #[clap(short, long, value_parser)]
    high_poly: String,
}

fn main() {
    let args = Args::parse();
}
