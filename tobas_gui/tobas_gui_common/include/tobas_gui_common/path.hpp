#pragma once

#include <filesystem>

namespace gui
{
namespace common
{
static constexpr char kImuFilterDynamicParamFileName[] = "imu_filter_dynamic.yaml";
static constexpr char kObserverDynamicParamFileName[] = "observer_dynamic.yaml";
static constexpr char kControllerDynamicParamFileName[] = "controller_dynamic.yaml";
static constexpr char kRcTeleopDynamicParamFileName[] = "rc_teleop_dynamic.yaml";

/* FC側のTobasパッケージのパスを返す． */
std::filesystem::path getProjRemotePath(const std::filesystem::path& proj_path);

/* パスから拡張子を除いたTobasパッケージ名を抽出する． */
std::string getProjName(const std::filesystem::path& proj_path);

/* 拡張子を除くTobasメタパッケージ名を返す． */
std::string getProjMetaPkgName(const std::filesystem::path& proj_path);

/* 拡張子を除くTobas設定パッケージ名を返す． */
std::string getProjCfgPkgName(const std::filesystem::path& proj_path);

/* 拡張子を除くTobasユーザMsgパッケージ名を返す． */
std::string getProjUserMsgPkgName(const std::filesystem::path& proj_path);

/* 拡張子を除くTobasユーザC++パッケージ名を返す． */
std::string getProjUserCppPkgName(const std::filesystem::path& proj_path);

/* 拡張子を除くTobasユーザPythonパッケージ名を返す． */
std::string getProjUserPyPkgName(const std::filesystem::path& proj_path);

/* Tobasメタパッケージの絶対パスを返す． */
std::filesystem::path getProjMetaPkgPath(const std::filesystem::path& proj_path);

/* Tobas設定パッケージの絶対パスを返す． */
std::filesystem::path getProjCfgPkgPath(const std::filesystem::path& proj_path);

/* TobasユーザMsgパッケージの絶対パスを返す． */
std::filesystem::path getProjUserMsgPkgPath(const std::filesystem::path& proj_path);

/* TobasユーザC++パッケージの絶対パスを返す． */
std::filesystem::path getProjUserCppPkgPath(const std::filesystem::path& proj_path);

/* TobasユーザPythonパッケージの絶対パスを返す． */
std::filesystem::path getProjUserPyPkgPath(const std::filesystem::path& proj_path);

/* 設定パッケージのconfigディレクトリの絶対パスを返す． */
std::filesystem::path getProjCfgConfigDirPath(const std::filesystem::path& proj_path);

/* 設定パッケージのlaunchディレクトリの絶対パスを返す． */
std::filesystem::path getProjCfgLaunchDirPath(const std::filesystem::path& proj_path);

/* 設定パッケージのmeshesディレクトリの絶対パスを返す． */
std::filesystem::path getProjCfgMeshDirPath(const std::filesystem::path& proj_path);

/* 設定パッケージのurdfディレクトリの絶対パスを返す． */
std::filesystem::path getProjCfgUrdfDirPath(const std::filesystem::path& proj_path);

/* original.uadf の絶対パスを返す． */
std::filesystem::path getProjOriginalUadfPath(const std::filesystem::path& proj_path);

/* drone.xacro の絶対パスを返す． */
std::filesystem::path getProjXacroPath(const std::filesystem::path& proj_path);

/* drone.tbsdrn の絶対パスを返す． */
std::filesystem::path getProjTbsDrnPath(const std::filesystem::path& proj_path);

/* ssh_endpoint.yaml の絶対パスを返す． */
std::filesystem::path getProjSshEndpointPath(const std::filesystem::path& proj_path);

/* imu_filter_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getProjImuFiltDynParamsPath(const std::filesystem::path& proj_path);

/* observer_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getProjObsvDynParamsPath(const std::filesystem::path& proj_path);

/* controller_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getProjCtrlDynParamsPath(const std::filesystem::path& proj_path);

/* rc_teleop_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getProjRcTeleopDynParamsPath(const std::filesystem::path& proj_path);

/* バックアップディレクトリの絶対パスを返す． */
std::filesystem::path getProjBackupDirPath(const std::filesystem::path& proj_path);

/* バックアップ用設定ファイルの絶対パスを返す． */
std::filesystem::path getProjBackupSettingsPath(const std::filesystem::path& proj_path);
}  // namespace common
}  // namespace gui
