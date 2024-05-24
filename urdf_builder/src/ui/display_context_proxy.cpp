#include "../../include/urdf_builder/ui/display_context_proxy.hpp"

using namespace rviz;

namespace urdf_builder
{
namespace ui
{
DisplayContextProxy::DisplayContextProxy(
  Ogre::SceneManager* scene_manager,
  SelectionManager* selection_manager)
  : scene_manager_(scene_manager), selection_manager_(selection_manager)
{
}

Ogre::SceneManager* DisplayContextProxy::getSceneManager() const
{
  return scene_manager_;
}

WindowManagerInterface* DisplayContextProxy::getWindowManager() const
{
  return nullptr;
}

SelectionManager* DisplayContextProxy::getSelectionManager() const
{
  return selection_manager_;
}

FrameManager* DisplayContextProxy::getFrameManager() const
{
  return nullptr;
}

QString DisplayContextProxy::getFixedFrame() const
{
  return "";
}

uint64_t DisplayContextProxy::getFrameCount() const
{
  return 0;
}

DisplayFactory* DisplayContextProxy::getDisplayFactory() const
{
  return nullptr;
}

ros::CallbackQueueInterface* DisplayContextProxy::getUpdateQueue()
{
  return nullptr;
}

ros::CallbackQueueInterface* DisplayContextProxy::getThreadedQueue()
{
  return nullptr;
}

void DisplayContextProxy::handleChar(QKeyEvent*, RenderPanel*)
{
}

void DisplayContextProxy::handleMouseEvent(const ViewportMouseEvent&)
{
}

ToolManager* DisplayContextProxy::getToolManager() const
{
  return nullptr;
}

ViewManager* DisplayContextProxy::getViewManager() const
{
  return nullptr;
}

DisplayGroup* DisplayContextProxy::getRootDisplayGroup() const
{
  return nullptr;
}

uint32_t DisplayContextProxy::getDefaultVisibilityBit() const
{
  return 0;
}

BitAllocator* DisplayContextProxy::visibilityBits()
{
  return nullptr;
}

void DisplayContextProxy::setStatus(const QString&)
{
}

void DisplayContextProxy::queueRender()
{
}
}  // namespace ui
}  // namespace urdf_builder
