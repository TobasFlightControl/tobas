# tobas_rapidcsv_vendor

This package provides the header-only [rapidcsv](https://github.com/d99kris/rapidcsv) CSV parser
as an exported CMake interface target.

## Vendored release

- Upstream tag: [`v8.99`](https://github.com/d99kris/rapidcsv/releases/tag/v8.99)
- Upstream commit: `68f57cc6c83d5e0992904398822453489d8dfac1`
- Source archive SHA-256: `fbbf738483e31e788dc7b37af5a899db7314be416400f7d87b9d771dd5ebac03`
- License: BSD-3-Clause; see `vendor/rapidcsv/LICENSE`

Only the public `src/rapidcsv.h` header is vendored.

## Updating the vendored source

1. Find the latest stable tag on the [upstream releases page](https://github.com/d99kris/rapidcsv/releases/latest),
   and set `UPSTREAM_TAG` below to that exact tag.
2. From this package directory, download and extract the corresponding source archive:

   ```bash
   UPSTREAM_TAG=v8.99
   UPDATE_DIR=$(mktemp -d)
   curl -L \
     -o "${UPDATE_DIR}/rapidcsv-${UPSTREAM_TAG}.tar.gz" \
     "https://github.com/d99kris/rapidcsv/archive/refs/tags/${UPSTREAM_TAG}.tar.gz"
   tar -xzf "${UPDATE_DIR}/rapidcsv-${UPSTREAM_TAG}.tar.gz" -C "${UPDATE_DIR}"
   sha256sum "${UPDATE_DIR}/rapidcsv-${UPSTREAM_TAG}.tar.gz"
   ```

3. Replace the public header and license:

   ```bash
   UPSTREAM_DIR="${UPDATE_DIR}/rapidcsv-${UPSTREAM_TAG#v}"
   install -m 0644 "${UPSTREAM_DIR}/src/rapidcsv.h" vendor/rapidcsv/rapidcsv.h
   install -m 0644 "${UPSTREAM_DIR}/LICENSE" vendor/rapidcsv/LICENSE
   ```

4. Update the tag, commit, and archive SHA-256 in the **Vendored release** section. Obtain the tag commit with:

   ```bash
   git ls-remote https://github.com/d99kris/rapidcsv.git "refs/tags/${UPSTREAM_TAG}" "refs/tags/${UPSTREAM_TAG}^{}"
   ```

5. Review the upstream changes and license, then validate the package and its consumer:

   ```bash
   pre-commit run --files CMakeLists.txt package.xml README.md
   colcon build --packages-up-to tobas_setup_assistant
   ```
