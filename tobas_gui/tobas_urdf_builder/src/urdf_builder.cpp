#include "tobas_urdf_builder/urdf_builder.hpp"

#include <filesystem>

#include <QVBoxLayout>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace urdf_builder
{
URDFBuilder::URDFBuilder() : rviz_manager_("rviz_urdf_builder")
{
  const fs::path pkg_path(ament_index_cpp::get_package_share_directory("tobas_urdf_builder"));
  const auto config_path = pkg_path / "config/urdf_builder.rviz";
  rviz_manager_.initialize(QString::fromStdString(config_path));

  const auto rows = new QVBoxLayout();
  setLayout(rows);
  rows->addWidget(rviz_manager_.widget());
}

void URDFBuilder::reset()
{
  rviz_manager_.resetTime();
}
}  // namespace urdf_builder
}  // namespace gui
