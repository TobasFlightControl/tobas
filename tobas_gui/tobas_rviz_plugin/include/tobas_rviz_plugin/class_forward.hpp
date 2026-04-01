// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./declare_ptr.hpp"

/* Macro that forward declares a class and defines the respective smartpointers through TOBAS_DECLARE_PTR. */
#define TOBAS_CLASS_FORWARD(C)                                                                                         \
  class C;                                                                                                             \
  TOBAS_DECLARE_PTR(C, C)

/* Like TOBAS_CLASS_FORWARD, but forward declares the type as a struct instead of a class. */
#define TOBAS_STRUCT_FORWARD(C)                                                                                        \
  struct C;                                                                                                            \
  TOBAS_DECLARE_PTR(C, C)
