// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QString>

namespace tobas
{
namespace gui
{
namespace cmn
{
class ProjectPaths
{
public:
  explicit ProjectPaths(const QString& proj_path = "");

  const QString& getProjPath() const;
  void setProjPath(const QString& proj_path);

  /* Return the path of the Tobas package on the FC side. */
  QString remoteProjPath() const;

  /* Extract the Tobas package name without extension from the path. */
  QString projName() const;

  /* Return the Tobas meta-package name without extension. */
  QString metaPkgName() const;

  /* Return the Tobas config package name without extension. */
  QString cfgPkgName() const;

  /* Return the Tobas user Msg package name without extension. */
  QString userMsgPkgName() const;

  /* Return the Tobas user C++ package name without extension. */
  QString userCppPkgName() const;

  /* Return the Tobas user Python package name without extension. */
  QString userPyPkgName() const;

  /* Return the absolute path of the Tobas meta-package. */
  QString metaPkgPath() const;

  /* Return the absolute path of the Tobas config package. */
  QString cfgPkgPath() const;

  /* Return the absolute path of the Tobas user Msg package. */
  QString userMsgPkgPath() const;

  /* Return the absolute path of the Tobas user C++ package. */
  QString userCppPkgPath() const;

  /* Return the absolute path of the Tobas user Python package. */
  QString userPyPkgPath() const;

  /* Return the absolute path of the config directory in the config package. */
  QString cfgConfigDirPath() const;

  /* Return the absolute path of the launch directory in the config package. */
  QString cfgLaunchDirPath() const;

  /* Return the absolute path of the meshes directory in the config package. */
  QString cfgMeshDirPath() const;

  /* Return the absolute path of the urdf directory in the config package. */
  QString cfgUrdfDirPath() const;

  /* Return the absolute path of original.uadf. */
  QString originalUadfPath() const;

  /* Return the absolute path of drone.xacro. */
  QString xacroPath() const;

  /* Return the absolute path of drone.tbsdrn. */
  QString tbsdrnPath() const;

  /* Return the absolute path of network.yaml. */
  QString networkConfigPath() const;

  /* Return the absolute path of imu_filter_dynamic.yaml. */
  QString imuFiltDynParamsPath() const;

  /* Return the absolute path of rpm_control_dynamic.yaml. */
  QString rpmCtrlDynParamsPath() const;

  /* Return the absolute path of observer_dynamic.yaml. */
  QString obsvDynParamsPath() const;

  /* Return the absolute path of controller_dynamic.yaml. */
  QString ctrlDynParamsPath() const;

  /* Return the absolute path of rc_teleop_dynamic.yaml. */
  QString rcTeleopDynParamsPath() const;

  /* Return the absolute path of the backup directory. */
  QString projBackupDirPath() const;

  /* Return the absolute path of the backup config file. */
  QString backupSettingsPath() const;

  /* Return the path of the version file. */
  QString versionPath() const;

private:
  QString proj_path_;
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
