#pragma once

#include <QObject>

#include <hardware_interface/hardware_info.hpp>

#include <tobas_kdl/tree.hpp>

namespace gui
{
namespace setup_assistant
{
namespace hw_interface
{
enum type_t : int
{
  // Valid (>= 0)
  POSITION = 0,
  VELOCITY = 1,
  EFFORT = 2,

  // Invalid (< 0)
  NONE = -1,
  UNKNOWN = -2,
};

static constexpr char kPositionInterface[] = "hardware_interface/PositionJointInterface";
static constexpr char kVelocityInterface[] = "hardware_interface/VelocityJointInterface";
static constexpr char kEffortInterface[] = "hardware_interface/EffortJointInterface";
}  // namespace hw_interface

class RobotInfo : public QObject
{
  Q_OBJECT

Q_SIGNALS:
  void loaded();

public:
  explicit RobotInfo();

  bool loadFromPath(const std::string& path);

  const kdl::Tree& tree() const;
  const hardware_interface::HardwareInfo& hardware() const;

  const std::string& robotName() const;

  hw_interface::type_t hardwareInterface(const std::string& jnt_name) const;

private:
  kdl::Tree tree_;
  hardware_interface::HardwareInfo hardware_;
};
}  // namespace setup_assistant
}  // namespace gui
