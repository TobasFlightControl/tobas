// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <QApplication>

#include <tobas_gui_common/argument.hpp>
#include <tobas_gui_common/version.hpp>
#include <tobas_qt_tools/debug.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>
#include <tobas_ros2_tools/async_node_manager.hpp>

#include "tobas_setup_assistant/setup_assistant.hpp"
#include "tobas_setup_assistant/util.hpp"

int main(int argc, char** argv)
{
  // X11を強制
  tobas::gui::cmn::NonRosArgumentParser arg_parser(argc, argv);
  if (!arg_parser.setPlatformXcb()) {
    std::cerr << "Failed to set display platform." << std::endl;
    return EXIT_FAILURE;
  }

  // コンソール出力に着色
  qInstallMessageHandler(tobas::qt::colorMessageHandler);

  // ノードを起動
  tobas::ros2::AsyncNodeManager node_manager(argc, argv, "tobas_setup_assistant");

  // GUIを表示
  QApplication qapp(arg_parser.argc(), arg_parser.argv());
  const auto title = "Tobas Setup Assistant (" + tobas::gui::cmn::Version::Current().toString() + ")";
  const auto icon_path = tobas::gui::sa::getPkgShareDir() / "resources/icon.png";
  const auto widget = new tobas::gui::sa::SetupAssistantWidget(node_manager.node());
  tobas::qt::MainWidget main(title, QString::fromStdString(icon_path), widget);
  main.show();

  // Ctrl+Cで即終了
  signal(SIGINT, [](int) { QApplication::quit(); });

  // アプリケーションの終了時に全てのノードを落とす
  const auto result = qapp.exec();
  rclcpp::shutdown();
  return result;
}
