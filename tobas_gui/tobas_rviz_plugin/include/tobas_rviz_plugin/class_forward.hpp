#pragma once

#include "./declare_ptr.hpp"

/**
 * \def TOBAS_CLASS_FORWARD
 * Macro that forward declares a class and defines the respective smartpointers through TOBAS_DECLARE_PTR.
 */
#define TOBAS_CLASS_FORWARD(C)                                                                                         \
  class C;                                                                                                             \
  TOBAS_DECLARE_PTR(C, C)

/**
 * \def TOBAS_STRUCT_FORWARD
 * Like TOBAS_CLASS_FORWARD, but forward declares the type as a struct
 * instead of a class.
 */
#define TOBAS_STRUCT_FORWARD(C)                                                                                        \
  struct C;                                                                                                            \
  TOBAS_DECLARE_PTR(C, C)
