#pragma once

#include <filesystem>

namespace gui
{
namespace common
{
/* FC側のTobasパッケージのパスを返す． */
std::filesystem::path getRemoteTBSPath(const std::filesystem::path& tbs_path);

/* パスから拡張子を除いたTobasパッケージ名を抽出する． */
std::string getTBSName(const std::filesystem::path& tbs_path);

/* 拡張子を除くTobasメタパッケージ名を返す． */
std::string getTBSMetaName(const std::filesystem::path& tbs_path);

/* 拡張子を除くTobas設定パッケージ名を返す． */
std::string getTBSConfigName(const std::filesystem::path& tbs_path);

/* 拡張子を除くTobasユーザMsgパッケージ名を返す． */
std::string getTBSUserMsgName(const std::filesystem::path& tbs_path);

/* 拡張子を除くTobasユーザC++パッケージ名を返す． */
std::string getTBSUserCppName(const std::filesystem::path& tbs_path);

/* 拡張子を除くTobasユーザPythonパッケージ名を返す． */
std::string getTBSUserPyName(const std::filesystem::path& tbs_path);

/* バックアップディレクトリの絶対パスを返す． */
std::filesystem::path getBackupPath(const std::filesystem::path& tbs_path);

/* Tobasメタパッケージの絶対パスを返す． */
std::filesystem::path getTBSMetaPath(const std::filesystem::path& tbs_path);

/* Tobas設定パッケージの絶対パスを返す． */
std::filesystem::path getTBSConfigPath(const std::filesystem::path& tbs_path);

/* TobasユーザMsgパッケージの絶対パスを返す． */
std::filesystem::path getTBSUserMsgPath(const std::filesystem::path& tbs_path);

/* TobasユーザC++パッケージの絶対パスを返す． */
std::filesystem::path getTBSUserCppPath(const std::filesystem::path& tbs_path);

/* TobasユーザPythonパッケージの絶対パスを返す． */
std::filesystem::path getTBSUserPyPath(const std::filesystem::path& tbs_path);

/* drone.tbsdrn の絶対パスを返す． */
std::filesystem::path getTBSDRNPath(const std::filesystem::path& tbs_path);

/* drone.xacro の絶対パスを返す． */
std::filesystem::path getModifiedURDFPath(const std::filesystem::path& tbs_path);

/* meshディレクトリの絶対パスを返す． */
std::filesystem::path getMeshPath(const std::filesystem::path& tbs_path);

/* imu_filter_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getImuFilterDynamicParamsPath(const std::filesystem::path& tbs_path);

/* observer_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getObserverDynamicParamsPath(const std::filesystem::path& tbs_path);

/* controller_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getControllerDynamicParamsPath(const std::filesystem::path& tbs_path);

/* rc_teleop_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getRcTeleopDynamicParamsPath(const std::filesystem::path& tbs_path);

/* バックアップ用設定ファイルの絶対パスを返す． */
std::filesystem::path getSettingsPath(const std::filesystem::path& tbs_path);

/* バックアップ用オリジナルURDFの絶対パスを返す． */
std::filesystem::path getOriginalURDFPath(const std::filesystem::path& tbs_path);
}  // namespace common
}  // namespace gui
