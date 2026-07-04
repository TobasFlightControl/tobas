# Contributing to Tobas

This guide is for contributors preparing changes for review.
If you only want to install and use Tobas, start from the
[Tobas User Guide](https://tobasflightcontrol.github.io/tobas/latest/).

## Setup

See [Setup](./SETUP.md) for source checkout, PC build, and FC deployment instructions.

## Pre-Commit Check

Install dependencies:

```bash
$ sudo apt install -y pre-commit black clang-format cmake-format qt6-declarative-dev-tools
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

See [C++ Code Style](./tobas_examples/tobas_code_style/CPP_CODE_STYLE.md).

### Python

See [Python Code Style](./tobas_examples/tobas_code_style/PYTHON_CODE_STYLE.md).

### CMake

See [CMake Code Style](./tobas_examples/tobas_code_style/CMAKE_CODE_STYLE.md).

## Git

See [Git Guideline](./GIT_GUIDELINE.md).
