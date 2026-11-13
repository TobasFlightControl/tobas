// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_debug_msgs/msg/fixed_wing_controller_feedback.hpp>
#include <tobas_debug_msgs_adapter/multicopter_controller_feedback.hpp>
#include <tobas_debug_msgs_adapter/observer_feedback.hpp>

#include "./rosbag_recorder.hpp"

namespace tobas
{
void RosbagRecorderNode::registerDebugSubscribers()
{
  addTypeAdaptedMsgSub<tobas_debug_msgs::ObserverFeedback>(obsv_fb_, topic::kObsvFeedback);
  addStandardMsgSub<tobas_debug_msgs::msg::FixedWingControllerFeedback>(topic::kFWCtrlFeedback);
  addTypeAdaptedMsgSub<tobas_debug_msgs::MulticopterControllerFeedback>(mr_ctrl_fb_, topic::kMRCtrlFeedback);
}
}  // namespace tobas
