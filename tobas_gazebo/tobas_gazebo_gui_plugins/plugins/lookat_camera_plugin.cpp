#include <boost/polymorphic_pointer_cast.hpp>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/rendering/RenderingIface.hh>

#include <tobas_gazebo_tools/conversion.hpp>

#include "./lookat_camera_plugin.hpp"

using namespace std;

namespace gazebo
{
LookAtCameraPlugin::LookAtCameraPlugin()
{
}

void LookAtCameraPlugin::LoadConfig(const tinyxml2::XMLElement* elem)
{
  if (title.empty())
    title = "LookAt Camera Plugin";

  if (elem)
  {
    // TODO: Get XML parameters
  }

  gz::gui::App()->findChild<gz::gui::MainWindow*>()->installEventFilter(this);
}

bool LookAtCameraPlugin::eventFilter(QObject* obj, QEvent* event)
{
  if (event->type() == gz::gui::events::Render::kType)
    onRender();

  return QObject::eventFilter(obj, event);
}

void LookAtCameraPlugin::onRender()
{
  lock_guard<mutex> lock(mutex_);

  if (!scene_)
  {
    scene_ = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene_)
      return;

    this->initialize();
  }

  if (!camera_)
    return;

  // Fixed camera position in world frame
  // TODO: 正しい位置を取得
  gz::math::Vector3d camera_pos(-5., 0., 1.7);

  // Direction vector
  const auto dir = (tar_pos_ - camera_pos).Normalized();

  // Compute yaw/pitch from direction vector
  const auto yaw = atan2(dir.Y(), dir.X());
  const auto pitch = -asin(dir.Z());

  // Roll = 0, so construct quaternion(roll, pitch, yaw)
  const gz::math::Quaterniond camera_rot(0., pitch, yaw);

  // Construct the new camera pose
  const gz::math::Pose3d camera_pose(camera_pos, camera_rot);

  // Update camera pose in the scene (world coordinates)
  camera_->SetWorldPose(camera_pose);
}

void LookAtCameraPlugin::initialize()
{
  // Attach to the first camera we find
  for (size_t i = 0; i < scene_->NodeCount(); ++i)
  {
    const auto camera = dynamic_pointer_cast<gz::rendering::Camera>(scene_->NodeByIndex(i));
    if (camera)
    {
      camera_ = camera;
      gzdbg << "LookAtCameraPlugin is moving camera [" << camera_->Name() << "]" << endl;
      break;
    }
  }

  if (!camera_)
  {
    gzerr << "Camera is not available." << endl;
    return;
  }

  node_.Subscribe("/gui/look_at_position", &LookAtCameraPlugin::lookAtPositionCb, this);
}

void LookAtCameraPlugin::lookAtPositionCb(const gz::msgs::Vector3d& msg)
{
  lock_guard<mutex> lock(mutex_);

  vector3dMsgToGz(msg, tar_pos_);
}
}  // namespace gazebo

GZ_ADD_PLUGIN(gazebo::LookAtCameraPlugin, gz::gui::Plugin)
