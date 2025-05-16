#include "tobas_control_system/mission_planner/system_info.hpp"

#include <QStandardPaths>

namespace gui
{
namespace gcs
{
SystemInfo::SystemInfo(QObject* parent) : QObject(parent)
{
  home_dir_ = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
}

QString SystemInfo::modelName() const
{
  return "SystemInfo";
}

QString SystemInfo::homeDirectory() const
{
  return home_dir_;
}
}  // namespace gcs
}  // namespace gui
