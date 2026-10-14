// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <csignal>

#include <QApplication>

#include <tobas_gui_common/argument.hpp>
#include <tobas_gui_common/version.hpp>
#include <tobas_qt_tools/debug.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>

#include "tobas_urdf_builder/urdf_builder.hpp"
#include "tobas_urdf_builder/util.hpp"

int main(int _argc, char** _argv)
{
  // Force X11.
  tobas::gui::cmn::NonRosArgumentParser arg_parser(_argc, _argv);
  if (!arg_parser.setPlatformXcb()) {
    std::cerr << "Failed to set display platform.\n";
    return EXIT_FAILURE;
  }

  // Colorize console output.
  qInstallMessageHandler(tobas::qt::colorMessageHandler);

  // Start the node so that RViz-related nodes receive the namespace and other arguments.
  rclcpp::init(_argc, _argv);

  // Show the GUI.
  QApplication qapp(arg_parser.argc(), arg_parser.argv());
  QApplication::setOrganizationName("tobas");
  QApplication::setApplicationName("urdf_builder");
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
