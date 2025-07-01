#pragma once

#include <filesystem>

namespace gui
{
namespace common
{
/* FC側のTobasパッケージのパスを返す． */
std::filesystem::path getProjRemotePath(const std::filesystem::path& tbs_path);

/* パスから拡張子を除いたTobasパッケージ名を抽出する． */
std::string getProjName(const std::filesystem::path& tbs_path);

/* 拡張子を除くTobasメタパッケージ名を返す． */
std::string getProjMetaPkgName(const std::filesystem::path& tbs_path);

/* 拡張子を除くTobas設定パッケージ名を返す． */
std::string getProjCfgPkgName(const std::filesystem::path& tbs_path);

/* 拡張子を除くTobasユーザMsgパッケージ名を返す． */
std::string getProjUserMsgPkgName(const std::filesystem::path& tbs_path);

/* 拡張子を除くTobasユーザC++パッケージ名を返す． */
std::string getProjUserCppPkgName(const std::filesystem::path& tbs_path);

/* 拡張子を除くTobasユーザPythonパッケージ名を返す． */
std::string getProjUserPyPkgName(const std::filesystem::path& tbs_path);

/* Tobasメタパッケージの絶対パスを返す． */
std::filesystem::path getProjMetaPkgPath(const std::filesystem::path& tbs_path);

/* Tobas設定パッケージの絶対パスを返す． */
std::filesystem::path getProjCfgPkgPath(const std::filesystem::path& tbs_path);

/* TobasユーザMsgパッケージの絶対パスを返す． */
std::filesystem::path getProjUserMsgPkgPath(const std::filesystem::path& tbs_path);

/* TobasユーザC++パッケージの絶対パスを返す． */
std::filesystem::path getProjUserCppPkgPath(const std::filesystem::path& tbs_path);

/* TobasユーザPythonパッケージの絶対パスを返す． */
std::filesystem::path getProjUserPyPkgPath(const std::filesystem::path& tbs_path);

/* 設定パッケージのconfigディレクトリの絶対パスを返す． */
std::filesystem::path getProjCfgConfigDirPath(const std::filesystem::path& tbs_path);

/* 設定パッケージのlaunchディレクトリの絶対パスを返す． */
std::filesystem::path getProjCfgLaunchDirPath(const std::filesystem::path& tbs_path);

/* 設定パッケージのmeshesディレクトリの絶対パスを返す． */
std::filesystem::path getProjCfgMeshDirPath(const std::filesystem::path& tbs_path);

/* 設定パッケージのurdfディレクトリの絶対パスを返す． */
std::filesystem::path getProjCfgUrdfDirPath(const std::filesystem::path& tbs_path);

/* drone.xacro の絶対パスを返す． */
std::filesystem::path getProjXacroPath(const std::filesystem::path& tbs_path);

/* drone.tbsdrn の絶対パスを返す． */
std::filesystem::path getProjTbsDrnPath(const std::filesystem::path& tbs_path);

/* imu_filter_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getProjImuFiltDynParamsPath(const std::filesystem::path& tbs_path);

/* observer_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getProjObsvDynParamsPath(const std::filesystem::path& tbs_path);

/* controller_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getProjCtrlDynParamsPath(const std::filesystem::path& tbs_path);

/* rc_teleop_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getProjRcTeleopDynParamsPath(const std::filesystem::path& tbs_path);

/* バックアップディレクトリの絶対パスを返す． */
std::filesystem::path getProjBackupDirPath(const std::filesystem::path& tbs_path);

/* バックアップ用設定ファイルの絶対パスを返す． */
std::filesystem::path getProjBackupSettingsPath(const std::filesystem::path& tbs_path);

/* バックアップ用オリジナルURDFの絶対パスを返す． */
std::filesystem::path getProjBackupUrdfPath(const std::filesystem::path& tbs_path);
}  // namespace common
}  // namespace gui
