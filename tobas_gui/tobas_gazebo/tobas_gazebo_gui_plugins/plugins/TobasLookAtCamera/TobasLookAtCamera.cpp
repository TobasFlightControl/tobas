#include "./TobasLookAtCamera.hpp"

#include <boost/polymorphic_pointer_cast.hpp>
#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_msg.hpp>

namespace gazebo
{
TobasLookAtCamera::TobasLookAtCamera()
{
}

void TobasLookAtCamera::LoadConfig(const tinyxml2::XMLElement* elem)
{
  if (title.empty()) {
    title = "LookAt Camera Plugin";
  }

  if (elem) {
    // TODO: Get XML parameters
  }

  gz::gui::App()->findChild<gz::gui::MainWindow*>()->installEventFilter(this);
}

bool TobasLookAtCamera::eventFilter(QObject* obj, QEvent* event)
{
  if (event->type() == gz::gui::events::Render::kType) {
    onRender();
  }

  return super::eventFilter(obj, event);
}

void TobasLookAtCamera::onRender()
{
  const std::lock_guard lock(mutex_);

  if (!scene_) {
    scene_ = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene_) {
      return;
    }

    initialize();
  }

  if (!camera_) {
    return;
  }

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

void TobasLookAtCamera::initialize()
{
  // Attach to the first camera we find
  for (size_t i = 0; i < scene_->NodeCount(); ++i) {
    const auto camera = std::dynamic_pointer_cast<gz::rendering::Camera>(scene_->NodeByIndex(i));
    if (camera) {
      camera_ = camera;
      gzdbg << "TobasLookAtCamera is moving camera [" << camera_->Name() << "]" << std::endl;
      break;
    }
  }

  if (!camera_) {
    gzerr << "Camera is not available." << std::endl;
    return;
  }

  node_.Subscribe(kGzCameraLookAtTopic, &TobasLookAtCamera::lookAtPositionCb, this);
}

void TobasLookAtCamera::lookAtPositionCb(const gz::msgs::Vector3d& msg)
{
  const std::lock_guard lock(mutex_);
  vector3dMsgToGz(msg, tar_pos_);
}
}  // namespace gazebo

GZ_ADD_PLUGIN(gazebo::TobasLookAtCamera, gz::gui::Plugin)
