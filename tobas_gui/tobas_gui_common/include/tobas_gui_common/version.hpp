#pragma once

#include <filesystem>

#include <QString>

namespace gui
{
namespace cmn
{
QString currentVersion();

class Version
{
  static constexpr char kMajorKey[] = "major";
  static constexpr char kMinorKey[] = "minor";
  static constexpr char kPatchKey[] = "patch";

public:
  int major = -1;
  int minor = -1;
  int patch = -1;

  void setToCurrent();

  bool isCompatible() const;

  bool load(const std::filesystem::path& path);
  bool save(const std::filesystem::path& path) const;
};
}  // namespace cmn
}  // namespace gui
