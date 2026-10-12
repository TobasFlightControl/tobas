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
  void finished(bool success, const QString& message);

public:
  explicit ColconBuildThread(const QString& pkg_path, const QString& ws_path, const colcon::BuildOptions& options)
    : options_(options), pkg_path_(pkg_path), ws_path_(ws_path)
  {
  }

  void run() override
  {
    if (const auto result = colcon::build(pkg_path_.toStdString(), ws_path_.toStdString(), options_); !result) {
      Q_EMIT finished(false, QString::fromStdString(result.error()));
      return;
    }

    Q_EMIT finished(true, "");
  }

private:
  const colcon::BuildOptions options_;
  const QString pkg_path_;
  const QString ws_path_;
};
}  // namespace

std::expected<void, QString>
colconBuild(const QString& pkg_path, const QString& ws_path, const colcon::BuildOptions& options)
{
  ColconBuildThread thread(pkg_path, ws_path, options);
  const auto [success, message] = qt::startThreadAndWait(thread, &ColconBuildThread::finished);
  if (!success) {
    return std::unexpected(message);
  }

  return {};
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas

#include "colcon.moc"
