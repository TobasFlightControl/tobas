#include <QStandardPaths>

#include "tobas_mission_planner/system_info.hpp"

namespace gui
{
namespace mission_planner
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
}  // namespace mission_planner
}  // namespace gui
