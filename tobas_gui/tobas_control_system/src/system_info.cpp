#include <QStandardPaths>

#include "tobas_control_system/system_info.hpp"

namespace gui
{
namespace control_system
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
}  // namespace control_system
}  // namespace gui
