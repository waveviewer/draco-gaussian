# Bundled third-party dependencies

Draco Gaussian builds without fetching anything: the dependencies that upstream
Draco keeps as Git submodules are checked in here at fixed revisions. The
build uses the same paths as upstream, so no CMake option is needed.

| Directory    | Project                                              | Revision   | Version | License                |
| ------------ | ---------------------------------------------------- | ---------- | ------- | ---------------------- |
| `eigen`      | [Eigen](https://gitlab.com/libeigen/eigen)            | `3147391`  | 3.4.0   | MPL2, BSD (see `COPYING.*`) |
| `filesystem` | [gulrak/filesystem](https://github.com/gulrak/filesystem) | `a07dded`  | 1.5.10  | MIT                    |
| `googletest` | [GoogleTest](https://github.com/google/googletest)    | `703bd9c`  | 1.10.0  | BSD-3-Clause           |
| `tinygltf`   | [tinygltf](https://github.com/syoyo/tinygltf)         | `a11f6e1`  | -       | MIT                    |

Each project keeps its own license file in its directory. Those licenses apply
to the files in these directories; the rest of the repository uses the Apache
License 2.0 (see `../LICENSE`).

## What was kept

To keep the repository small, files that Draco never builds were removed:

- `eigen`: only `Eigen/` and `unsupported/` plus the license and readme files.
  The Eigen tests, benchmarks, documentation and build files were removed.
- `tinygltf`: only the headers Draco includes (`tiny_gltf.h`, `json.hpp`,
  `stb_image.h`, `stb_image_write.h`) plus the license and readme. The example
  programs and their sample models were removed.
- `googletest` and `filesystem` are complete upstream checkouts without their
  CI configurations.

If you need the complete upstream sources, check out the revisions listed
above.
