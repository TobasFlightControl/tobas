#pragma once

#include <memory>
#include <unordered_set>
#include <rviz_common/display_context.hpp>

#include "../view_model/urdf_view_model.hpp"

namespace gui
{
namespace urdf_builder
{
namespace ogre
{
class OgreController
{
  static constexpr float kAxesLength = 0.1;
  static constexpr float kAxesRadius = 0.01;
  static constexpr float kCharHeight = 0.03;  // モデルビューに表示される文字のサイズ
  static constexpr float kHighlightR = 0.;
  static constexpr float kHighlightG = 1.;
  static constexpr float kHighlightB = 0.;

public:
  using SharedPtr = std::shared_ptr<OgreController>;

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
}  // namespace urdf_builder
}  // namespace gui
