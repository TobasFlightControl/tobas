// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <QApplication>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_gui_common/argument.hpp>
#include <tobas_gui_common/version.hpp>
#include <tobas_qt_tools/debug.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>

#include <tobas_urdf_builder/urdf_builder.hpp>
#include <tobas_urdf_builder/util.hpp>

int main(int argc, char** argv)
{
  // Force X11.
  tobas::gui::cmn::NonRosArgumentParser arg_parser(argc, argv);
  if (!arg_parser.setPlatformXcb()) {
    std::cerr << "Failed to set display platform." << std::endl;
    return EXIT_FAILURE;
  }

  // Colorize console output.
  qInstallMessageHandler(tobas::qt::colorMessageHandler);

  // Start the node so namespace and other arguments are reflected in Rviz-related nodes.
  rclcpp::init(argc, argv);

  // Show the GUI.
  QApplication qapp(arg_parser.argc(), arg_parser.argv());
  const auto title = "Tobas URDF Builder (" + tobas::gui::cmn::Version::Current().toString() + ")";
  const auto icon_path = tobas::gui::ub::getPkgShareDir() / "resources/icon.png";
  const auto widget = new tobas::gui::ub::URDFBuilder();
  tobas::qt::MainWidget main(title, QString::fromStdString(icon_path), widget);
  main.show();

  // Exit immediately on Ctrl+C.
  signal(SIGINT, [](int) { QApplication::quit(); });

  // Shut down all nodes when the application exits.
  const auto result = qapp.exec();
  rclcpp::shutdown();
  return result;
}
