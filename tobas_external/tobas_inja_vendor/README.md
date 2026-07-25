# tobas_inja_vendor

This package provides the header-only [inja](https://github.com/pantor/inja) template engine
as an exported CMake interface target.

## Vendored release

- Upstream tag: [`v3.5.0`](https://github.com/pantor/inja/releases/tag/v3.5.0)
- Upstream commit: `7d1b4600b68595085a949743331c2e5673f511ea`
- Source archive SHA-256: `a5f0266673c59028eab6ceeddd8b862c70abfeb32fb7a5387c16bf46f3269ab2`
- License: MIT; see `vendor/inja/LICENSE`

Only the public headers under `include/inja` are vendored.
The upstream copy of nlohmann/json is intentionally omitted;
this package uses the `nlohmann-json-dev` rosdep key instead.

## Updating the vendored source

1. Find the latest stable tag on the [upstream releases page](https://github.com/pantor/inja/releases/latest),
   and set `UPSTREAM_TAG` below to that exact tag.

2. From this package directory, download and extract the corresponding source archive:

   ```bash
   UPSTREAM_TAG=v3.5.0
   UPDATE_DIR=$(mktemp -d)
   curl -L \
     -o "${UPDATE_DIR}/inja-${UPSTREAM_TAG}.tar.gz" \
     "https://github.com/pantor/inja/archive/refs/tags/${UPSTREAM_TAG}.tar.gz"
   tar -xzf "${UPDATE_DIR}/inja-${UPSTREAM_TAG}.tar.gz" -C "${UPDATE_DIR}"
   sha256sum "${UPDATE_DIR}/inja-${UPSTREAM_TAG}.tar.gz"
   ```

3. Replace the public headers and license. Do not copy `third_party/include/nlohmann`:

   ```bash
   UPSTREAM_DIR="${UPDATE_DIR}/inja-${UPSTREAM_TAG#v}"
   rsync -a --delete "${UPSTREAM_DIR}/include/inja/" vendor/inja/include/inja/
   install -m 0644 "${UPSTREAM_DIR}/LICENSE" vendor/inja/LICENSE
   ```

4. Update the tag, commit, and archive SHA-256 in the **Vendored release** section. Obtain the tag commit with:

   ```bash
   git ls-remote https://github.com/pantor/inja.git "refs/tags/${UPSTREAM_TAG}" "refs/tags/${UPSTREAM_TAG}^{}"
   ```

5. Review the upstream changes and license, then validate the package and its consumers:

   ```bash
   pre-commit run --files CMakeLists.txt package.xml README.md
   colcon build --packages-up-to tobas_setup_assistant tobas_bootmedia_config
   ```
