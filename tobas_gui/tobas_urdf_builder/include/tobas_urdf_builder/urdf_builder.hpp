#pragma once

#include <tobas_qt_tools/rviz.hpp>

namespace gui
{
namespace urdf_builder
{
class URDFBuilder : public QWidget
{
  Q_OBJECT

public:
  explicit URDFBuilder();

  void reset();

private:
  qt::RvizFrameManager rviz_manager_;
};
}  // namespace urdf_builder
}  // namespace gui
