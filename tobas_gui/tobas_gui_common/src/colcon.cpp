// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/colcon.hpp"

#include <QThread>

#include <tobas_qt_tools/thread.hpp>

namespace tobas
{
namespace gui
{
namespace cmn
{
namespace
{
class ColconBuildThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success);

public:
  explicit ColconBuildThread(colcon::Colcon& colcon, const QString& pkg_path, const QString& ws_path)
    : colcon_(colcon), pkg_path_(pkg_path), ws_path_(ws_path)
  {
  }

  void run() override
  {
    const auto res = colcon_.build(pkg_path_.toStdString(), ws_path_.toStdString());
    Q_EMIT finished(res);
  }

private:
  colcon::Colcon& colcon_;
  const QString pkg_path_;
  const QString ws_path_;
};
}  // namespace

bool colconBuild(colcon::Colcon& colcon, const QString& pkg_path, const QString& ws_path)
{
  ColconBuildThread thread(colcon, pkg_path, ws_path);
  return std::get<0>(qt::startThreadAndWait(thread, &ColconBuildThread::finished));
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas

#include "colcon.moc"
