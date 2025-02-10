#pragma once

#include <QThread>

#include <tobas_gui_common/local_package_builder.hpp>

namespace gui
{
namespace sa
{
class BuildPackageThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& output);

public:
  explicit BuildPackageThread();

  void run() override;

  void setPackagePath(const QString& tbs_path);

private:
  QString tbs_path_;
  common::LocalPackageBuilder package_builder_;
};
}  // namespace sa
}  // namespace gui
