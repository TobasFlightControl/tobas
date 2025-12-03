#pragma once

#include <expected>
#include <filesystem>

namespace ros2
{
/* pathが属するパッケージのパスを返す． */
std::expected<std::filesystem::path, std::string> getPackagePathOf(const std::filesystem::path& path);

/* pathが属するパッケージのpackage.xmlのnameを読み取る． */
std::expected<std::string, std::string> getPackageNameOf(const std::filesystem::path& path);

/* pathが属するパッケージのパスから最も近いsrcディレクトリの親ディレクトリを返す． */
std::expected<std::filesystem::path, std::string> estimateWorkspaceOf(const std::filesystem::path& path);

/* パッケージがビルド・インストール済みかどうかを判定する． */
bool isAlreadyBuiltAndInstalled(const std::filesystem::path& pkg_path);
}  // namespace ros2
