// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "./TobasRenderInfo.hpp"

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>

#include <tobas_gazebo_common/constants.hpp>

using namespace std::chrono_literals;
namespace ch = std::chrono;

namespace tobas
{
namespace gazebo
{
TobasRenderInfo::TobasRenderInfo()
{
}

void TobasRenderInfo::LoadConfig(const tinyxml2::XMLElement*)
{
  if (title.empty()) {
    title = "Rendering Information Plugin";
  }

  fps_pub_ = node_.Advertise<gz::msgs::Double>(gazebo::kGzRenderFpsTopic);

  gz::gui::App()->findChild<gz::gui::MainWindow*>()->installEventFilter(this);
}

bool TobasRenderInfo::eventFilter(QObject* obj, QEvent* event)
{
  if (event->type() == gz::gui::events::Render::kType) {
    onRender();
  }

  return super::eventFilter(obj, event);
}

void TobasRenderInfo::onRender()
{
  const auto now = ch::steady_clock::now();

  if (!scene_) {
    scene_ = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene_) {
      return;
    }

    t_last_tick_ = t_last_pub_ = now;
    return;
  }

  if (now <= t_last_tick_) {
    gzwarn << "The steady clock is not advancing." << std::endl;
    return;
  }

  const auto dt = ch::duration_cast<ch::duration<double>>(now - t_last_tick_).count();  // [s]
  t_last_tick_ = now;

  const auto fps_inst = 1.0 / dt;
  if (std::isnan(fps_ema_)) {
    fps_ema_ = fps_inst;
  }
  else {
    constexpr double kAlpha = 0.05;
    fps_ema_ = kAlpha * fps_inst + (1.0 - kAlpha) * fps_ema_;
  }

  constexpr auto kPublishPeriod = 1000ms;
  if (now - t_last_pub_ >= kPublishPeriod) {
    fps_msg_.set_data(fps_ema_);
    fps_pub_.Publish(fps_msg_);

    t_last_pub_ = now;
  }
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(tobas::gazebo::TobasRenderInfo, gz::gui::Plugin)
