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
    SelectionManager* selection_manager)
    : scene_manager_(scene_manager), selection_manager_(selection_manager)
  {
  }

  Ogre::SceneManager* getSceneManager() const override
  {
    return scene_manager_;
  }

  WindowManagerInterface* getWindowManager() const override
  {
    return nullptr;
  }

  SelectionManager* getSelectionManager() const override
  {
    return selection_manager_;
  }

  FrameManager* getFrameManager() const override
  {
    return nullptr;
  }

  QString getFixedFrame() const override
  {
    return "";
  }

  uint64_t getFrameCount() const override
  {
    return 0;
  }

  DisplayFactory* getDisplayFactory() const override
  {
    return nullptr;
  }

  ros::CallbackQueueInterface* getUpdateQueue() override
  {
    return nullptr;
  }

  ros::CallbackQueueInterface* getThreadedQueue() override
  {
    return nullptr;
  }

  void handleChar(QKeyEvent*, RenderPanel*) override
  {
  }

  void handleMouseEvent(const ViewportMouseEvent&) override
  {
  }

  ToolManager* getToolManager() const override
  {
    return nullptr;
  }

  ViewManager* getViewManager() const override
  {
    return nullptr;
  }

  DisplayGroup* getRootDisplayGroup() const override
  {
    return nullptr;
  }

  uint32_t getDefaultVisibilityBit() const override
  {
    return 0;
  }

  BitAllocator* visibilityBits() override
  {
    return nullptr;
  }

  void setStatus(const QString&) override
  {
  }

public Q_SLOTS:

  void queueRender() override
  {
  }

private:
  Ogre::SceneManager* scene_manager_;
  SelectionManager* selection_manager_;
};
}  // namespace rviz
