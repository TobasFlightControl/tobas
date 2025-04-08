#include <boost/polymorphic_pointer_cast.hpp>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/rendering/RenderingIface.hh>

#include <tobas_gazebo_tools/conversion.hpp>

#include "./LookAtCamera.hpp"

using namespace std;

namespace gazebo
{
LookAtCamera::LookAtCamera()
{
}

void LookAtCamera::LoadConfig(const tinyxml2::XMLElement* elem)
{
  if (title.empty())
    title = "LookAt Camera Plugin";

  if (elem)
  {
    // TODO: Get XML parameters
  }

  gz::gui::App()->findChild<gz::gui::MainWindow*>()->installEventFilter(this);
}

bool LookAtCamera::eventFilter(QObject* obj, QEvent* event)
{
  if (event->type() == gz::gui::events::Render::kType)
    onRender();

  return QObject::eventFilter(obj, event);
}

void LookAtCamera::onRender()
{
  lock_guard<mutex> lock(mutex_);

  if (!scene_)
  {
    scene_ = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene_)
      return;

    initialize();
  }

  if (!camera_)
    return;

  // Get camera position
  const auto camera_pos = camera_->WorldPosition();

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

void LookAtCamera::initialize()
{
  // Attach to the first camera we find
  for (size_t i = 0; i < scene_->NodeCount(); ++i)
  {
    const auto camera = dynamic_pointer_cast<gz::rendering::Camera>(scene_->NodeByIndex(i));
    if (camera)
    {
      camera_ = camera;
      gzdbg << "LookAtCamera is moving camera [" << camera_->Name() << "]" << endl;
      break;
    }
  }

  if (!camera_)
  {
    gzerr << "Camera is not available." << endl;
    return;
  }

  node_.Subscribe("/gui/look_at_position", &LookAtCamera::lookAtPositionCb, this);
}

void LookAtCamera::lookAtPositionCb(const gz::msgs::Vector3d& msg)
{
  lock_guard<mutex> lock(mutex_);

  vector3dMsgToGz(msg, tar_pos_);
}
}  // namespace gazebo

GZ_ADD_PLUGIN(gazebo::LookAtCamera, gz::gui::Plugin)
