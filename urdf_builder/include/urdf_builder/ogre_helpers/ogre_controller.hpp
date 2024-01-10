#pragma once

#include <memory>
#include <unordered_set>

#include "../view_model/urdf_view_model.hpp"

namespace rviz
{
class Robot;
class Axes;

using RobotPtr = std::shared_ptr<Robot>;
using AxesPtr = std::shared_ptr<Axes>;
}  // namespace rviz

namespace urdf_builder
{
namespace ogre_helpers
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
  explicit OgreController(rviz::VisualizationManager* visualizationManager);

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

private:
  struct PImpl;
  std::unique_ptr<PImpl> pimpl_;
  std::unordered_set<std::string> highlighted_links_;
  std::unordered_set<std::string> hidden_links_;
};

using OgreControllerPtr = std::shared_ptr<OgreController>;
}  // namespace ogre_helpers
}  // namespace urdf_builder
