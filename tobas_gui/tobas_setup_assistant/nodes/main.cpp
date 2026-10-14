// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <csignal>

#include <QApplication>

#include <tobas_gui_common/argument.hpp>
#include <tobas_gui_common/version.hpp>
#include <tobas_qt_tools/debug.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>
#include <tobas_ros2_tools/async_node_manager.hpp>

#include "tobas_setup_assistant/setup_assistant.hpp"
#include "tobas_setup_assistant/util.hpp"

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

  // Start the node.
  tobas::ros2::AsyncNodeManager node_manager(_argc, _argv, "tobas_setup_assistant");

  // Show the GUI.
  QApplication qapp(arg_parser.argc(), arg_parser.argv());
  QApplication::setOrganizationName("tobas");
  QApplication::setApplicationName("setup_assistant");
  const auto title = "Tobas Setup Assistant (" + tobas::gui::cmn::Version::Current().toString() + ")";
  const auto icon_path = tobas::gui::sa::getPkgShareDir() / "resources/icon.png";
  const auto widget = new tobas::gui::sa::SetupAssistantWidget(node_manager.node());
  tobas::qt::MainWidget main(title, QString::fromStdString(icon_path), widget);
  main.show();

  // Exit immediately on Ctrl+C.
  signal(SIGINT, [](int) { QApplication::quit(); });

  // Shut down all nodes when the application exits.
  const auto result = qapp.exec();
  rclcpp::shutdown();
  return result;
}
