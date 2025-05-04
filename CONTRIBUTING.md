# Contributing to Tobas

## Pre-Commit Check

Install dependencies:

```
$ sudo apt install -y black clang-format cmake-format
```

Check all:

```
$ pre-commit run -a
```

To check every time you commit in git (optional):

```
$ pre-commit install
```

To update hook repository versions:

```
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
