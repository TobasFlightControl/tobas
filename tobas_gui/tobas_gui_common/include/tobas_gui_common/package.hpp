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

/* 拡張子を除くTobasユーザパッケージ名を返す． */
std::string getTBSUserName(const std::filesystem::path& tbs_path);

/* PC上のTobasメタパッケージの絶対パスを返す． */
std::filesystem::path getTBSMetaPath(const std::filesystem::path& tbs_path);

/* PC上のTobas設定パッケージの絶対パスを返す． */
std::filesystem::path getTBSConfigPath(const std::filesystem::path& tbs_path);

/* PC上のTobasユーザパッケージの絶対パスを返す． */
std::filesystem::path getTBSUserPath(const std::filesystem::path& tbs_path);

/* PC上の drone.tbsdrn の絶対パスを返す． */
std::filesystem::path getTBSDRNPath(const std::filesystem::path& tbs_path);

/* PC上の drone.xacro の絶対パスを返す． */
std::filesystem::path getModifiedURDFPath(const std::filesystem::path& tbs_path);

/* PC上の meshディレクトリの絶対パスを返す． */
std::filesystem::path getMeshPath(const std::filesystem::path& tbs_path);

/* PC上の controller_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getControllerDynamicParamsPath(const std::filesystem::path& tbs_path);

/* PC上の observer_dynamic.yaml の絶対パスを返す． */
std::filesystem::path getObserverDynamicParamsPath(const std::filesystem::path& tbs_path);

/* PC上のバックアップ用設定ファイルの絶対パスを返す． */
std::filesystem::path getSettingsPath(const std::filesystem::path& tbs_path);

/* PC上のバックアップ用オリジナルURDFの絶対パスを返す． */
std::filesystem::path getOriginalURDFPath(const std::filesystem::path& tbs_path);
}  // namespace common
}  // namespace gui
