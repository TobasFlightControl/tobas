// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace cyclonedds
{
namespace elem
{
static constexpr char kCycloneDDS[] = "CycloneDDS";
static constexpr char kDomain[] = "Domain";
static constexpr char kGeneral[] = "General";
static constexpr char kInterfaces[] = "Interfaces";
static constexpr char kNIF[] = "NetworkInterface";
static constexpr char kRedundantNetworking[] = "RedundantNetworking";
static constexpr char kSharedMemory[] = "SharedMemory";
static constexpr char kEnable[] = "Enable";
static constexpr char kLogLevel[] = "LogLevel";
}  // namespace elem

namespace attr
{
static constexpr char kName[] = "name";
static constexpr char kPriority[] = "priority";
static constexpr char kMulticast[] = "multicast";
}  // namespace attr
}  // namespace cyclonedds
}  // namespace tobas
