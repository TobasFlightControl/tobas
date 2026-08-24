// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <csignal>

#include <QApplication>

#include <tobas_gui_common/argument.hpp>
#include <tobas_gui_common/version.hpp>
#include <tobas_qt_tools/debug.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>

#include "tobas_gcs/gcs.hpp"
#include "tobas_gcs/util.hpp"

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

  // Show the GUI.
  QApplication qapp(arg_parser.argc(), arg_parser.argv());
  QApplication::setOrganizationName("tobas");
  QApplication::setApplicationName("gcs");
  const auto title = "Tobas (" + tobas::gui::cmn::Version::Current().toString() + ")";
  const auto icon_path = tobas::gui::gcs::getPkgShareDir() / "resources/icon.png";
  const auto widget = new tobas::gui::gcs::GroundControlStationWidget(argc, argv);
  tobas::qt::MainWidget main(title, QString::fromStdString(icon_path), widget);
  main.show();

  // Exit immediately on Ctrl+C.
  signal(SIGINT, [](int) { QApplication::quit(); });

  return qapp.exec();
}
