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

  /* FC側のTobasパッケージのパスを返す． */
  std::filesystem::path remoteProjPath() const;

  /* パスから拡張子を除いたTobasパッケージ名を抽出する． */
  std::string projName() const;

  /* 拡張子を除くTobasメタパッケージ名を返す． */
  std::string metaPkgName() const;

  /* 拡張子を除くTobas設定パッケージ名を返す． */
  std::string cfgPkgName() const;

  /* 拡張子を除くTobasユーザMsgパッケージ名を返す． */
  std::string userMsgPkgName() const;

  /* 拡張子を除くTobasユーザC++パッケージ名を返す． */
  std::string userCppPkgName() const;

  /* 拡張子を除くTobasユーザPythonパッケージ名を返す． */
  std::string userPyPkgName() const;

  /* Tobasメタパッケージの絶対パスを返す． */
  std::filesystem::path metaPkgPath() const;

  /* Tobas設定パッケージの絶対パスを返す． */
  std::filesystem::path cfgPkgPath() const;

  /* TobasユーザMsgパッケージの絶対パスを返す． */
  std::filesystem::path userMsgPkgPath() const;

  /* TobasユーザC++パッケージの絶対パスを返す． */
  std::filesystem::path userCppPkgPath() const;

  /* TobasユーザPythonパッケージの絶対パスを返す． */
  std::filesystem::path userPyPkgPath() const;

  /* 設定パッケージのconfigディレクトリの絶対パスを返す． */
  std::filesystem::path cfgConfigDirPath() const;

  /* 設定パッケージのlaunchディレクトリの絶対パスを返す． */
  std::filesystem::path cfgLaunchDirPath() const;

  /* 設定パッケージのmeshesディレクトリの絶対パスを返す． */
  std::filesystem::path cfgMeshDirPath() const;

  /* 設定パッケージのurdfディレクトリの絶対パスを返す． */
  std::filesystem::path cfgUrdfDirPath() const;

  /* original.uadf の絶対パスを返す． */
  std::filesystem::path originalUadfPath() const;

  /* drone.xacro の絶対パスを返す． */
  std::filesystem::path xacroPath() const;

  /* drone.tbsdrn の絶対パスを返す． */
  std::filesystem::path tbsdrnPath() const;

  /* ssh.yaml の絶対パスを返す． */
  std::filesystem::path sshConfigPath() const;

  /* network.yaml の絶対パスを返す． */
  std::filesystem::path networkConfigPath() const;

  /* imu_filter_dynamic.yaml の絶対パスを返す． */
  std::filesystem::path imuFiltDynParamsPath() const;

  /* rpm_control_dynamic.yaml の絶対パスを返す． */
  std::filesystem::path rpmCtrlDynParamsPath() const;

  /* observer_dynamic.yaml の絶対パスを返す． */
  std::filesystem::path obsvDynParamsPath() const;

  /* controller_dynamic.yaml の絶対パスを返す． */
  std::filesystem::path ctrlDynParamsPath() const;

  /* rc_teleop_dynamic.yaml の絶対パスを返す． */
  std::filesystem::path rcTeleopDynParamsPath() const;

  /* バックアップディレクトリの絶対パスを返す． */
  std::filesystem::path projBackupDirPath() const;

  /* バックアップ用設定ファイルの絶対パスを返す． */
  std::filesystem::path backupSettingsPath() const;

  /* バージョンファイルのパスを返す． */
  std::filesystem::path versionPath() const;

private:
  std::filesystem::path proj_path_;
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
