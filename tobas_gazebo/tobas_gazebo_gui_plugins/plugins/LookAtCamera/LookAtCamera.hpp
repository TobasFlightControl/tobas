#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Camera.hh>
#include <gz/msgs/vector3d.pb.h>

using namespace std;

namespace gazebo
{
/* cf. gz-gui/src/plugins/camera_tracking/CameraTracking.cc */
class LookAtCamera : public gz::gui::Plugin
{
  Q_OBJECT

  using self = LookAtCamera;
  using super = gz::gui::Plugin;

public:
  explicit LookAtCamera();

  void LoadConfig(const tinyxml2::XMLElement* elem) override;

private:
  bool eventFilter(QObject* obj, QEvent* event) override;

  void onRender();
  void initialize();

  void lookAtPositionCb(const gz::msgs::Vector3d& msg);

  gz::transport::Node node_;

  gz::rendering::ScenePtr scene_;
  gz::rendering::CameraPtr camera_;

  gz::math::Vector3d tar_pos_ = gz::math::Vector3d::Zero;

  std::mutex mutex_;
};
}  // namespace gazebo
