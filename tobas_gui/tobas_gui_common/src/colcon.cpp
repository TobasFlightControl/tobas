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
  void finished(bool _success, const QString& _message);

public:
  explicit ColconBuildThread(const QString& _pkg_path, const QString& _ws_path, const colcon::BuildOptions& _options)
    : options_(_options), pkg_path_(_pkg_path), ws_path_(_ws_path)
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
colconBuild(const QString& _pkg_path, const QString& _ws_path, const colcon::BuildOptions& _options)
{
  ColconBuildThread thread(_pkg_path, _ws_path, _options);
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
