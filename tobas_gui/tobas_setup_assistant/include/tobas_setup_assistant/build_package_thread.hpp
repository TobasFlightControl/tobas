#pragma once

#include <QThread>

#include <tobas_tools/package_builder.hpp>

namespace gui
{
namespace setup_assistant
{
class BuildPackageThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& output);

public:
  explicit BuildPackageThread(const QString& tbs_path);

  void run() override;

private:
  const QString tbs_path_;
  tobas::PackageBuilder package_builder_;
};
}  // namespace setup_assistant
}  // namespace gui
