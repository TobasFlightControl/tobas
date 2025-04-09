#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace sim
{
class WorldWidget_Standard : public WorldWidget_Base
{
  Q_OBJECT

  using self = WorldWidget_Standard;
  using super = WorldWidget_Base;

public:
  explicit WorldWidget_Standard();

  std::filesystem::path worldPath() const override;
  void setContentsEnabled(bool enable) override;

private:
  qt::ComboBox* combo_box_;

  static std::filesystem::path worldDirectoryPath();
};
}  // namespace sim
}  // namespace gui
