# CMake Code Style

Use `tobas_cmake` for shared C++ build requirements.
Declare it as a `buildtool_depend` in `package.xml`, so colcon builds it before configuring the package.

```cmake
cmake_minimum_required(VERSION 3.31)
project(tobas_example)

find_package(ament_cmake REQUIRED)
find_package(tobas_cmake REQUIRED)
tobas_default_build_type()

add_library(${PROJECT_NAME} STATIC src/example.cpp)
tobas_target_defaults(${PROJECT_NAME})
```

Call `tobas_target_defaults()` for each compiled library or executable, including examples and tests.
It selects C++23 without compiler extensions, enables position independent code,
and applies the Tobas warning policy to that target's C++ sources.
It leaves dependency targets, interface libraries, and consumer compiler flags alone.
Keep package-specific options alongside the target that needs them.

`tobas_default_build_type()` selects Release only for single-configuration builds without an explicit build type.
It preserves Debug and multi-configuration generators.

The `NO_SWITCH_ENUM` option preserves the existing `tobas_components_rt` exception
until that package can enable `-Wswitch-enum`.
