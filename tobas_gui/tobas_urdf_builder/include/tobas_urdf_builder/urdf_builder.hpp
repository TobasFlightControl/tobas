#pragma once

#include <QWidget>

#include <tobas_rviz_wrapper/rviz.hpp>

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
  rviz::RvizFrameManager rviz_manager_;
};
}  // namespace urdf_builder
}  // namespace gui
