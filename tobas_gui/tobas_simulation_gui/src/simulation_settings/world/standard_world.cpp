#include "tobas_simulation_gui/simulation_settings/world/standard_world.hpp"

#include <QHBoxLayout>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
WorldWidget_Standard::WorldWidget_Standard()
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  combo_box_ = new qt::ComboBox();
  combo_box_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  cols->addWidget(combo_box_);

  // Add world names
  for (const auto& entry : fs::recursive_directory_iterator(worldDirectoryPath())) {
    if (entry.is_regular_file() && entry.path().extension() == ".world") {
      const auto world_name = entry.path().stem().string();
      combo_box_->addItem(QString::fromStdString(world_name));
    }
  }

  combo_box_->sort();
  combo_box_->setCurrentText("flight_training");
}

fs::path WorldWidget_Standard::worldPath() const
{
  const auto world_name = combo_box_->currentText().toStdString();
  return (worldDirectoryPath() / world_name).replace_extension(".world");
}

fs::path WorldWidget_Standard::worldDirectoryPath()
{
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory("tobas_gazebo_sim"));
  return pkg_path / "worlds";
}
}  // namespace sim
}  // namespace gui
