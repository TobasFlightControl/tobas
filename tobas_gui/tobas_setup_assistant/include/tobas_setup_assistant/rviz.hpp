// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <tobas_kdl/tree.hpp>
#include <tobas_rviz_wrapper/rviz.hpp>
#include <tobas_uadf/model.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
class RvizWidget : public QWidget
{
  Q_OBJECT

  using self = RvizWidget;

public:
  explicit RvizWidget(const uadf::Model& _uadf, const kdl::Tree& _tree);

  void updateInternalDataStructures();

  void heightLink(const QString& _link_name);
  void unheightLink(const QString& _link_name);

private:
  const uadf::Model& uadf_;
  const kdl::Tree& tree_;

  rviz::RvizFrameManager rviz_manager_;
  rviz_common::Display* display_;

  rviz_common::properties::BoolProperty* enable_visual_;
  rviz_common::properties::BoolProperty* enable_collision_;
  rviz_common::properties::BoolProperty* enable_inertia_;
  rviz_common::properties::StringProperty* highlight_link_;
  rviz_common::properties::StringProperty* unhighlight_link_;
  rviz_common::properties::BoolProperty* reload_;

  QString highlighted_link_;

  void resetOrbitView();

private Q_SLOTS:
  void onVisualBoxToggled(bool _checked);
  void onCollisionBoxToggled(bool _checked);
  void onInertiaBoxToggled(bool _checked);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
