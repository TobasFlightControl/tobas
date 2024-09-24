#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QVBoxLayout>

#include <tobas_qt_tools/rviz/util.hpp>

#include "tobas_gui_core/urdf_builder.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace core
{
URDFBuilder::URDFBuilder(rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node_if)
{
  const auto pkg_path = fs::path(ament_index_cpp::get_package_share_directory("urdf_builder"));
  const auto config_path = pkg_path / "config/urdf_builder.rviz";
  frame_ = qt::createRvizFrame(rviz_node_if, QString::fromStdString(config_path));

  const auto rows = new QVBoxLayout();
  setLayout(rows);
  rows->addWidget(frame_);
}
}  // namespace core
}  // namespace gui
