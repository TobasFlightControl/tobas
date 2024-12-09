#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QVBoxLayout>

#include "tobas_gui_core/urdf_builder.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace core
{
URDFBuilder::URDFBuilder() : rviz_manager_("rviz_urdf_builder")
{
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory("urdf_builder"));
  const auto config_path = pkg_path / "config/urdf_builder.rviz";
  rviz_manager_.initialize(QString::fromStdString(config_path));

  const auto rows = new QVBoxLayout();
  setLayout(rows);
  rows->addWidget(rviz_manager_.frame());
}
}  // namespace core
}  // namespace gui
