// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/project_paths.hpp"

#include <QDir>
#include <QFileInfo>

#include <tobas_std_tools/check.hpp>

#include "tobas_gui_common/constants.hpp"

namespace tobas
{
namespace gui
{
namespace cmn
{
ProjectPaths::ProjectPaths(const QString& proj_path) : proj_path_(proj_path)
{
}

const QString& ProjectPaths::getProjPath() const
{
  return proj_path_;
}

void ProjectPaths::setProjPath(const QString& proj_path)
{
  proj_path_ = proj_path;
}

QString ProjectPaths::remoteProjPath() const
{
  return QDir("/etc/tobas/colcon_ws/src").filePath(QFileInfo(proj_path_).fileName());
}

QString ProjectPaths::projName() const
{
  const QFileInfo info(proj_path_);
  TOBAS_CHECK('.' + info.suffix() == kProjectExtension);
  return info.completeBaseName();
}

QString ProjectPaths::metaPkgName() const
{
  return projName();
}

QString ProjectPaths::cfgPkgName() const
{
  return projName() + "_config";
}

QString ProjectPaths::userMsgPkgName() const
{
  return projName() + "_user_msgs";
}

QString ProjectPaths::userCppPkgName() const
{
  return projName() + "_user_cpp";
}

QString ProjectPaths::userPyPkgName() const
{
  return projName() + "_user_py";
}

QString ProjectPaths::metaPkgPath() const
{
  return QDir(proj_path_).filePath(metaPkgName());
}

QString ProjectPaths::cfgPkgPath() const
{
  return QDir(proj_path_).filePath(cfgPkgName());
}

QString ProjectPaths::userMsgPkgPath() const
{
  return QDir(proj_path_).filePath(userMsgPkgName());
}

QString ProjectPaths::userCppPkgPath() const
{
  return QDir(proj_path_).filePath(userCppPkgName());
}

QString ProjectPaths::userPyPkgPath() const
{
  return QDir(proj_path_).filePath(userPyPkgName());
}

QString ProjectPaths::cfgConfigDirPath() const
{
  return QDir(cfgPkgPath()).filePath("config");
}

QString ProjectPaths::cfgLaunchDirPath() const
{
  return QDir(cfgPkgPath()).filePath("launch");
}

QString ProjectPaths::cfgMeshDirPath() const
{
  return QDir(cfgPkgPath()).filePath("meshes");
}

QString ProjectPaths::cfgUrdfDirPath() const
{
  return QDir(cfgPkgPath()).filePath("urdf");
}

QString ProjectPaths::originalUadfPath() const
{
  return QDir(cfgUrdfDirPath()).filePath("original.uadf");
}

QString ProjectPaths::xacroPath() const
{
  return QDir(cfgUrdfDirPath()).filePath("drone.xacro");
}

QString ProjectPaths::tbsdrnPath() const
{
  return QDir(cfgConfigDirPath()).filePath("drone.tbsdrn");
}

QString ProjectPaths::networkConfigPath() const
{
  return QDir(cfgConfigDirPath()).filePath("network.yaml");
}

QString ProjectPaths::imuFiltDynParamsPath() const
{
  return QDir(cfgConfigDirPath()).filePath("imu_filter_dynamic.yaml");
}

QString ProjectPaths::rpmCtrlDynParamsPath() const
{
  return QDir(cfgConfigDirPath()).filePath("rpm_control_dynamic.yaml");
}

QString ProjectPaths::obsvDynParamsPath() const
{
  return QDir(cfgConfigDirPath()).filePath("observer_dynamic.yaml");
}

QString ProjectPaths::ctrlDynParamsPath() const
{
  return QDir(cfgConfigDirPath()).filePath("controller_dynamic.yaml");
}

QString ProjectPaths::rcTeleopDynParamsPath() const
{
  return QDir(cfgConfigDirPath()).filePath("rc_teleop_dynamic.yaml");
}

QString ProjectPaths::projBackupDirPath() const
{
  return QDir(proj_path_).filePath("backup");
}

QString ProjectPaths::backupSettingsPath() const
{
  return QDir(projBackupDirPath()).filePath("settings.yaml");
}

QString ProjectPaths::versionPath() const
{
  return QDir(proj_path_).filePath("version");
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
