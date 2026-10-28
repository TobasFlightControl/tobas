# tobas_qwt_vendor

Builds the unmodified Qwt 6.3.0 sources against Qt 6. Ubuntu 26.04's Qwt development
package is built against Qt 5 and cannot be linked into the Qt 6 RViz/Gazebo GUI.
The library has a Tobas-specific filename to coexist with system Qwt installations.

Source: https://sourceforge.net/projects/qwt/files/qwt/6.3.0/qwt-6.3.0.tar.bz2

Archive SHA-256: `dcb085896c28aaec5518cbc08c0ee2b4e60ada7ac929d82639f6189851a6129a`.
Only `src/` and `COPYING` are vendored. See `vendor/COPYING` for the Qwt License 1.0.
The CMake build excludes the legacy QGLWidget canvas, following upstream's Qt 6 build.
