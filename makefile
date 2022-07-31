test_tutte:
	cargo run --release --bin tutte -- --mesh data/beetle.obj
render:
	cargo run --release --bin render -- --mesh data/spot/spot_triangulated.obj \
  --texture data/spot/spot_texture.png --kind diffuse
