#pragma once

#include <rviz/display_context.h>

namespace rviz
{
class DisplayContextProxy : public DisplayContext
{
  Q_OBJECT

public:
  explicit DisplayContextProxy(
    Ogre::SceneManager* scene_manager,
    SelectionManager* selection_manager);

  Ogre::SceneManager* getSceneManager() const override;
  WindowManagerInterface* getWindowManager() const override;
  SelectionManager* getSelectionManager() const override;
  FrameManager* getFrameManager() const override;
  QString getFixedFrame() const override;
  uint64_t getFrameCount() const override;
  DisplayFactory* getDisplayFactory() const override;
  ros::CallbackQueueInterface* getUpdateQueue() override;
  ros::CallbackQueueInterface* getThreadedQueue() override;
  void handleChar(QKeyEvent*, RenderPanel*) override;
  void handleMouseEvent(const ViewportMouseEvent&) override;
  ToolManager* getToolManager() const override;
  ViewManager* getViewManager() const override;
  DisplayGroup* getRootDisplayGroup() const override;
  uint32_t getDefaultVisibilityBit() const override;
  BitAllocator* visibilityBits() override;
  void setStatus(const QString&) override;

public Q_SLOTS:
  void queueRender() override;

private:
  Ogre::SceneManager* scene_manager_;
  SelectionManager* selection_manager_;
};
}  // namespace rviz
