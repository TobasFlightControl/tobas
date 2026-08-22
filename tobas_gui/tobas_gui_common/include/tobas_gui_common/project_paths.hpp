// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

namespace tobas
{
namespace gui
{
namespace cmn
{
class ProjectPaths
{
public:
  static constexpr char kImuFilterDynamicParamFileName[] = "imu_filter_dynamic.yaml";
  static constexpr char kRpmControlDynamicParamFileName[] = "rpm_control_dynamic.yaml";
  static constexpr char kObserverDynamicParamFileName[] = "observer_dynamic.yaml";
  static constexpr char kControllerDynamicParamFileName[] = "controller_dynamic.yaml";
  static constexpr char kRcTeleopDynamicParamFileName[] = "rc_teleop_dynamic.yaml";

  explicit ProjectPaths(const std::filesystem::path& proj_path = "");

  const std::filesystem::path& getProjPath() const;
  void setProjPath(const std::filesystem::path& proj_path);

  /* Return the path of the Tobas package on the FC side. */
  std::filesystem::path remoteProjPath() const;

  /* Extract the Tobas package name without extension from the path. */
  std::string projName() const;

  /* Return the Tobas meta-package name without extension. */
  std::string metaPkgName() const;

  /* Return the Tobas config package name without extension. */
  std::string cfgPkgName() const;

  /* Return the Tobas user Msg package name without extension. */
  std::string userMsgPkgName() const;

  /* Return the Tobas user C++ package name without extension. */
  std::string userCppPkgName() const;

  /* Return the Tobas user Python package name without extension. */
  std::string userPyPkgName() const;

  /* Return the absolute path of the Tobas meta-package. */
  std::filesystem::path metaPkgPath() const;

  /* Return the absolute path of the Tobas config package. */
  std::filesystem::path cfgPkgPath() const;

  /* Return the absolute path of the Tobas user Msg package. */
  std::filesystem::path userMsgPkgPath() const;

  /* Return the absolute path of the Tobas user C++ package. */
  std::filesystem::path userCppPkgPath() const;

  /* Return the absolute path of the Tobas user Python package. */
  std::filesystem::path userPyPkgPath() const;

  /* Return the absolute path of the config directory in the config package. */
  std::filesystem::path cfgConfigDirPath() const;

  /* Return the absolute path of the launch directory in the config package. */
  std::filesystem::path cfgLaunchDirPath() const;

  /* Return the absolute path of the meshes directory in the config package. */
  std::filesystem::path cfgMeshDirPath() const;

  /* Return the absolute path of the urdf directory in the config package. */
  std::filesystem::path cfgUrdfDirPath() const;

  /* Return the absolute path of original.uadf. */
  std::filesystem::path originalUadfPath() const;

  /* Return the absolute path of drone.xacro. */
  std::filesystem::path xacroPath() const;

  /* Return the absolute path of drone.tbsdrn. */
  std::filesystem::path tbsdrnPath() const;

  /* Return the absolute path of network.yaml. */
  std::filesystem::path networkConfigPath() const;

  /* Return the absolute path of imu_filter_dynamic.yaml. */
  std::filesystem::path imuFiltDynParamsPath() const;

  /* Return the absolute path of rpm_control_dynamic.yaml. */
  std::filesystem::path rpmCtrlDynParamsPath() const;

  /* Return the absolute path of observer_dynamic.yaml. */
  std::filesystem::path obsvDynParamsPath() const;

  /* Return the absolute path of controller_dynamic.yaml. */
  std::filesystem::path ctrlDynParamsPath() const;

  /* Return the absolute path of rc_teleop_dynamic.yaml. */
  std::filesystem::path rcTeleopDynParamsPath() const;

  /* Return the absolute path of the backup directory. */
  std::filesystem::path projBackupDirPath() const;

  /* Return the absolute path of the backup config file. */
  std::filesystem::path backupSettingsPath() const;

  /* Return the path of the version file. */
  std::filesystem::path versionPath() const;

private:
  std::filesystem::path proj_path_;
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
