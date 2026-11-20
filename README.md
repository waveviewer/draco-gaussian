# Draco Gaussian

A fork of [Google Draco](https://github.com/google/draco) that compresses
[3D Gaussian splatting](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/)
point clouds.

Draco is a library and a set of command line tools for compressing 3D meshes and
point clouds. A Gaussian splatting scene is a PLY file with one point per
Gaussian, plus attributes such as opacity, scale, rotation and spherical
harmonics coefficients. This fork teaches Draco's PLY reader and PLY encoder
about those attributes, so a scene can be encoded to a `.drc` file and converted
back to PLY without losing them.

## What this fork adds

The PLY reader maps the attribute layout written by the common Gaussian
splatting exporters onto Draco attributes, and the encoder gives each of them
its own quantization setting:

| PLY property    | Draco attribute                  | Quantization |
| --------------- | -------------------------------- | ------------ |
| `x` `y` `z`     | `POSITION`                       | 11 bits      |
| `nx` `ny` `nz`  | `NORMAL`                         | 8 bits       |
| `f_dc_0..2`     | `CUSTOM_ATTRIBUTE_SH_DC`         | 10 bits      |
| `f_rest_0..44`  | `CUSTOM_ATTRIBUTE_SH_REST`       | 8 bits       |
| `opacity`       | `CUSTOM_ATTRIBUTE_OPACITY`       | no quantization (float32) |
| `scale_0..2`    | `CUSTOM_ATTRIBUTE_SCALE`         | 8 bits       |
| `rot_0..3`      | `CUSTOM_ATTRIBUTE_ROTATION`      | 10 bits      |
| `xyz_disp_0..2` | `CUSTOM_ATTRIBUTE_DISPLACEMENT`  | 12 bits      |

The attribute names are stored in the Draco metadata, so the PLY encoder can
write the original property names back when a scene is decoded.

Behaviour worth knowing:

- Gaussian splatting PLY files are detected automatically. Ordinary PLY point
  clouds and meshes still load through the regular decoder.
- `draco_encoder` accepts vertex-only PLY files without the `-point_cloud` flag.
- The new attribute types use the numbers 16 to 21. A `.drc` file produced by
  this fork therefore cannot be decoded by stock Draco builds, which reject
  unknown attribute types. Use this fork on both sides.
- Supported layout: a single `vertex` element, `float32` attributes, and
  spherical harmonics up to degree 3 (45 `f_rest` values).

## Build

The build follows upstream Draco: CMake 3.12 or newer and a C++14 compiler.
The third-party dependencies are bundled in `third_party`, so a plain clone is
enough and no submodule download is needed.

```bash
git clone https://github.com/waveviewer/draco-gaussian.git
cd draco-gaussian
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The encoder and decoder end up in `build/`. See [BUILDING.md](BUILDING.md) for
the full list of build options.

## Usage

Encode a Gaussian splatting PLY file:

```bash
build/draco_encoder -i scene.ply -o scene.drc
```

Decode it back to PLY:

```bash
build/draco_decoder -i scene.drc -o scene.ply
```

The output PLY contains the same Gaussian attributes as the input, so it can be
opened by any Gaussian splatting viewer.

## Documentation

Upstream documentation applies to the shared code: see [docs](docs) and
[BUILDING.md](BUILDING.md). The Gaussian splatting additions are limited to the
files listed in [NOTICE](NOTICE).

## License

Licensed under the Apache License, Version 2.0, the same license as Google
Draco. See [LICENSE](LICENSE) for the full text and [NOTICE](NOTICE) for the
attribution and the list of modified files.

## Acknowledgements

This project is a derivative of [Google Draco](https://github.com/google/draco),
Copyright 2016 The Draco Authors. Thanks to the Draco team and to the authors of
the 3D Gaussian Splatting reference implementation for the file format
documented above.
