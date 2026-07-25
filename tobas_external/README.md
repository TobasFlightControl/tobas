# Tobas external dependencies

This directory contains ROS packages that expose third-party libraries to the Tobas workspace.
A dependency should use an operating-system or ROS binary package when one is available for every supported platform.
A small, header-only dependency without a suitable binary package is stored in a self-contained vendor package instead.

## Why Git submodules are not used

Git records a submodule as a commit reference rather than storing the referenced repository contents in the parent repository.
Consequently, a source archive of the parent repository contains the submodule entry but not the required headers.
Bloom and Debian source-package builds need all build inputs to be present in the released source,
and ROS build jobs must not depend on initializing submodules or downloading code from the network.

Keeping the required upstream files in a vendor package provides the following properties:

- Bloom source archives and Debian source packages are complete.
- Builds work without network access after system dependencies have been installed.
- The exact upstream tag, commit, archive checksum, and license are reviewable in the repository.
- A normal clone is sufficient; `git clone --recurse-submodules` is unnecessary.
- Local colcon builds and binary-package builds use the same source files.

Vendoring is limited to the files needed to build Tobas.
Each vendor package documents its upstream provenance and update
procedure:

- [`tobas_inja_vendor`](./tobas_inja_vendor/README.md)
- [`tobas_rapidcsv_vendor`](./tobas_rapidcsv_vendor/README.md)

Do not replace these packages with `FetchContent`, `ExternalProject`, or configure-time downloads.
Those approaches make offline and release builds depend on external network availability.
