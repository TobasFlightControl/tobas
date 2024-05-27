#pragma once

#include <rviz/display_context.h>

namespace urdf_builder
{
namespace ui
{
class DisplayContextProxy : public rviz::DisplayContext
{
  Q_OBJECT

public:
  explicit DisplayContextProxy(Ogre::SceneManager* scene_manager, rviz::SelectionManager* selection_manager);

  Ogre::SceneManager* getSceneManager() const override;
  rviz::WindowManagerInterface* getWindowManager() const override;
  rviz::SelectionManager* getSelectionManager() const override;
  rviz::FrameManager* getFrameManager() const override;
  QString getFixedFrame() const override;
  uint64_t getFrameCount() const override;
  rviz::DisplayFactory* getDisplayFactory() const override;
  ros::CallbackQueueInterface* getUpdateQueue() override;
  ros::CallbackQueueInterface* getThreadedQueue() override;
  void handleChar(QKeyEvent*, rviz::RenderPanel*) override;
  void handleMouseEvent(const rviz::ViewportMouseEvent&) override;
  rviz::ToolManager* getToolManager() const override;
  rviz::ViewManager* getViewManager() const override;
  rviz::DisplayGroup* getRootDisplayGroup() const override;
  uint32_t getDefaultVisibilityBit() const override;
  rviz::BitAllocator* visibilityBits() override;
  void setStatus(const QString&) override;

public Q_SLOTS:
  void queueRender() override;

private:
  Ogre::SceneManager* scene_manager_;
  rviz::SelectionManager* selection_manager_;
};
}  // namespace ui
}  // namespace urdf_builder
