# Contributing to Tobas

## Pre-Commit Check

Install dependencies:

```bash
$ sudo apt install -y pre-commit black clang-format cmake-format
```

To check all:

```bash
$ pre-commit run -a
```

To check every time you commit in git (optional):

```bash
$ pre-commit install
```

To update hook repository versions:

```bash
$ pre-commit autoupdate
```

## Code Style

### C++

See [C++ Code Style](./tobas_code_style/CPP_CODE_STYLE.md).

### Python

See [Python Code Style](./tobas_code_style/PYTHON_CODE_STYLE.md).

### CMake

See [CMake Code Style](./tobas_code_style/CMAKE_CODE_STYLE.md).

## Git

See [Git Guideline](./GIT_GUIDELINE.md).

## Commands

### Sync

Synchronize the Tobas repository on the PC with the FC.

```bash
user@host $ tobas_dev_tools/scripts/tobas_sync
```

### Build

```bash
# e.g. tobas_dev_tools/scripts/tobas_build_upto tobas
pi@tobas $ tobas_dev_tools/scripts/tobas_build_(debug, release)_(upto, above, select) ${package_name}
```

### Install

Install the release build in `/opt/tobas`.

```bash
pi@tobas $ tobas_dev_tools/scripts/tobas_install
```

### Restart

```bash
pi@tobas $ sudo systemctl restart tobas_real.target
```
