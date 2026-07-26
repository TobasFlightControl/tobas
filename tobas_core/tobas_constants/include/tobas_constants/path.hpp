// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
// Resource Path
// cf. XDG Base Directory Specification: https://specifications.freedesktop.org/basedir/
static constexpr char kConfigDirHome[] = "~/.local/share/tobas/config";
static constexpr char kConfigDirRoot[] = "/etc/tobas/config";
static constexpr char kColconWSPathHome[] = "~/.local/share/tobas/colcon_ws";
static constexpr char kColconWSPathRoot[] = "/etc/tobas/colcon_ws";
static constexpr char kRosbagDirHome[] = "~/.local/share/tobas/rosbag";
static constexpr char kRosbagDirRoot[] = "/etc/tobas/rosbag";
static constexpr char kProjectEnvPath[] = "/etc/tobas/config/project.env";
static constexpr char kCycloneddsConfigPath[] = "/etc/tobas/config/cyclonedds.xml";
static constexpr char kGuiLogDir[] = "~/.local/share/tobas/log";
static constexpr char kMissionDir[] = "~/.local/share/tobas/mission";
}  // namespace tobas
