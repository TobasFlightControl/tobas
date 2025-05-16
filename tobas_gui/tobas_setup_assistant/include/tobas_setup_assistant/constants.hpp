#pragma once

namespace gui
{
namespace sa
{
static constexpr char kPackageName[] = "tobas_setup_assistant";
static constexpr char kTitle[] = "Tobas Setup Assistant";

// ROS parameters
static constexpr char kRobotDescriptionParam[] = "robot_description";
static constexpr char kRobotDescriptionSemanticParam[] = "robot_description_semantic";

// Point Sizes
static constexpr int kTitlePSize = 18;
static constexpr int kLabelPSize = 12;
static constexpr int kBodyPSize = 9;

// Default Parameters
static constexpr int kDefaultNumFlightModes = 2;

// Common Descriptions
static constexpr char kCameraLinkDescription[] = "The name of the link to which the camera is attached.";
static constexpr char kCameraOffsetDescription[] = "The pose of the camera frame wrt. the the selected link frame.";
static constexpr char kSensorOffsetDescription[] = "The pose of the sensor frame wrt. the the drone root frame.";
}  // namespace sa
}  // namespace gui
