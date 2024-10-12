#pragma once

#include <tobas_qt_tools/rviz.hpp>

namespace gui
{
namespace core
{
class URDFBuilder : public QWidget
{
  Q_OBJECT

public:
  explicit URDFBuilder();

private:
  qt::RvizFrameManager rviz_manager_;
};
}  // namespace core
}  // namespace gui
