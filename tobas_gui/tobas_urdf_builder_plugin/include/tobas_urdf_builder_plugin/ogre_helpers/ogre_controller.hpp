// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>
#include <unordered_set>

#include <rviz_common/display_context.hpp>

#include "../view_model/urdf_view_model.hpp"

namespace tobas::gui::ub::ogre
{
class OgreController
{
public:
  explicit OgreController(rviz_common::DisplayContext* _context);
  ~OgreController();

  void update();

  void reload(const view_model::URDFViewModel& _vm);
  void reloadRobot(const view_model::URDFViewModel& _vm);
  void reloadAxes(const view_model::URDFViewModel& _vm);

  void highlight(const std::string& _link_name);
  void unhighlight(const std::string& _link_name);
  void unhighlightAll();
  void show(const std::string& _link_name);
  void hide(const std::string& _link_name);
  void setVisualVisible(bool _visible);
  void setCollisionVisible(bool _visible);
  void setInertiaVisible(bool _visible);

private:
  struct PImpl;
  std::unique_ptr<PImpl> pimpl_;
  std::unordered_set<std::string> highlighted_links_;
  std::unordered_set<std::string> hidden_links_;
};
}  // namespace tobas::gui::ub::ogre
