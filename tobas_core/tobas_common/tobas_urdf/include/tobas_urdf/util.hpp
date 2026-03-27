#pragma once

#include <filesystem>

namespace tobas
{
namespace urdf
{
/* URDF中のファイルの絶対パスを返す． */
std::filesystem::path resolveURI(const std::string& uri);
}  // namespace urdf
}  // namespace tobas
