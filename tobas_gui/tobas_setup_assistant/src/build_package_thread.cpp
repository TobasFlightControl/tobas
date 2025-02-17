#include "tobas_setup_assistant/build_package_thread.hpp"

namespace gui
{
namespace sa
{
BuildPackageThread::BuildPackageThread()
{
}

void BuildPackageThread::setPackagePath(const QString& tbs_path)
{
  tbs_path_ = tbs_path;
}

void BuildPackageThread::run()
{
  const auto success = package_builder_.build(tbs_path_.toStdString());
  const auto output = QString::fromStdString(package_builder_.getOutput());
  Q_EMIT finished(success, output);
}
}  // namespace sa
}  // namespace gui
