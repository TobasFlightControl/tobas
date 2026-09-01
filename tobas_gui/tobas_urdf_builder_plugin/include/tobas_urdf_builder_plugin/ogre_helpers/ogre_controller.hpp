// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>
#include <unordered_set>

#include <rviz_common/display_context.hpp>

#include "../view_model/urdf_view_model.hpp"

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ogre
{
class OgreController
{
public:
  explicit OgreController(rviz_common::DisplayContext* context);
  ~OgreController();

  void update();

  void reload(const view_model::URDFViewModel& vm);
  void reloadRobot(const view_model::URDFViewModel& vm);
  void reloadAxes(const view_model::URDFViewModel& vm);

  void highlight(const std::string& link_name);
  void unhighlight(const std::string& link_name);
  void unhighlightAll();
  void show(const std::string& link_name);
  void hide(const std::string& link_name);
  void setVisualVisible(bool visible);
  void setCollisionVisible(bool visible);
  void setInertiaVisible(bool visible);

private:
  struct PImpl;
  std::unique_ptr<PImpl> pimpl_;
  std::unordered_set<std::string> highlighted_links_;
  std::unordered_set<std::string> hidden_links_;
};
}  // namespace ogre
}  // namespace ub
}  // namespace gui
}  // namespace tobas
