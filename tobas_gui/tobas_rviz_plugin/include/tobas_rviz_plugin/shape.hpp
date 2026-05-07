// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <geometric_shapes/shapes.h>

namespace shapes
{
using ShapePtr = std::shared_ptr<Shape>;
using ShapeConstPtr = std::shared_ptr<const Shape>;
}  // namespace shapes
