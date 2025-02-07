#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tobas_simulation_gui/static_configuration/world/standard_world.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
WorldWidget_Standard::WorldWidget_Standard() : super("Standard World")
{
  combo_box_ = new qt::ComboBox();
  combo_box_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  cols_->addWidget(combo_box_);

  // Add worlds
  // TODO: "tobas_gazebo_sim/worlds/"以下のworldファイルを検索し，全てのベース名を選択肢に含める．
  combo_box_->addItem("basic");
}

fs::path WorldWidget_Standard::worldPath() const
{
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory("tobas_gazebo_sim"));
  const auto world_name = combo_box_->currentText().toStdString();
  return (pkg_path / "worlds" / world_name).replace_extension(".world");
}

void WorldWidget_Standard::setContentsEnabled(bool enable)
{
  combo_box_->setEnabled(enable);
}
}  // namespace sim
}  // namespace gui
