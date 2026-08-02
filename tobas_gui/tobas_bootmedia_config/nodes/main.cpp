// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <csignal>

#include <QApplication>

#include <tobas_gui_common/version.hpp>
#include <tobas_qt_tools/debug.hpp>
#include <tobas_qt_tools/widgets/main_widget.hpp>

#include "tobas_bootmedia_config/bootmedia_config.hpp"
#include "tobas_bootmedia_config/util.hpp"

int main(int argc, char** argv)
{
  qInstallMessageHandler(tobas::qt::colorMessageHandler);

  QApplication qapp(argc, argv);
  QApplication::setOrganizationName("tobas");
  QApplication::setApplicationName("bootmedia_config");

  const auto title = "Tobas Bootmedia Config (" + tobas::gui::cmn::Version::Current().toString() + ")";
  const auto icon_path = tobas::gui::bm::getPkgShareDir() / "resources/icon.png";
  const auto widget = new tobas::gui::bm::BootmediaConfigWidget();
  tobas::qt::MainWidget main(title, QString::fromStdString(icon_path), widget);
  main.show();

  signal(SIGINT, [](int) { QApplication::quit(); });

  const auto result = qapp.exec();
  return result;
}
