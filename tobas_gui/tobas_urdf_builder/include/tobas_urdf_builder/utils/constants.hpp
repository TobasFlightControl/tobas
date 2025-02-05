#pragma once

namespace urdf_builder
{
static constexpr char kPropertySection[] = "urdf_builder";
static constexpr char kError[] = "ERROR";
static constexpr float kDefaultRobotAlpha = 0.7;  // FIXME: 1以外だとMeshのときに反映されない
static constexpr bool kDefaultVisualVisible = true;
static constexpr bool kDefaultCollisionVisible = true;
static constexpr bool kDefaultInertiaVisible = false;
}  // namespace urdf_builder
