// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/settings.hpp"

#include <QHBoxLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/check.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
SettingsWidget::SettingsWidget(const uadf::Model& uadf, const kdl::Tree& tree, Signals& sig) : uadf_(uadf)
{
  stack_ = new qt::StackedWidget();
  navigation_ = new SettingsNavigationWidget();
  connect(navigation_, &SettingsNavigationWidget::currentEntryChanged, stack_, &QStackedWidget::setCurrentIndex);

  // Pages
  propulsion_system = new propulsion::PropulsionSystemWidget(uadf, sig);
  fixed_wing = new fw::FixedWingWidget(uadf);
  hardware = new hw::HardwareWidget(uadf, sig);
  remote_connection = new rc::RemoteConnectionWidget();
  observer = new ObserverWidget();
  controller = new ctrl::ControllerWidget();
  mission = new mission::MissionExecutorWidget();
  rc_input = new RcInputWidget();
  extra_joints = new ExtraJointsWidget(uadf, tree);
  failsafe = new FailsafeWidget();
  author_info = new AuthorInformationWidget();

  // Basic settings
  navigation_->addSection("BASIC SETTINGS");
  addPage(propulsion_system);
  addPage(fixed_wing);
  addPage(hardware);
  addPage(remote_connection);

  // Advanced settings
  navigation_->addSection("ADVANCED SETTINGS");
  addPage(observer);
  addPage(controller);
  addPage(mission);
  addPage(rc_input);
  addPage(extra_joints);
  addPage(failsafe);
  addPage(author_info);

  // Disable all pages.
  for (int i = 0; i < stack_->count(); ++i) {
    setPageEnabled(i, false);
  }

  // Layout
  const auto cols = new QHBoxLayout();
  cols->setContentsMargins(0, 0, 0, 0);
  cols->setSpacing(12);
  setLayout(cols);
  cols->addWidget(navigation_, 0);
  cols->addWidget(stack_, 1);
}

void SettingsWidget::updateInternalDataStructures()
{
  for (int i = 0; i < stack_->count(); ++i) {
    const auto page = qt::qPointerCast<BaseSettingWidget>(stack_->widget(i));
    page->updateInternalDataStructures();
    setPageEnabled(i, true);
  }

  // Disable settings when there are no rotors.
  if (uadf_.thrusts.empty()) {
    setPageEnabled(propulsion_system, false);
  }

  // Disable settings when there are no fixed wings.
  if (uadf_.control_surfaces.empty()) {
    setPageEnabled(fixed_wing, false);
  }

  // Disable settings when there are no extra joints.
  if (extra_joints->numJoints() == 0) {
    setPageEnabled(extra_joints, false);
  }

  // Default page
  setCurrentPage(0);
}

void SettingsWidget::setToDefaults()
{
  for (int i = 0; i < stack_->count(); ++i) {
    const auto page = qt::qPointerCast<BaseSettingWidget>(stack_->widget(i));
    page->setToDefaults();
  }
}

bool SettingsWidget::isValid()
{
  // Confirm that each setting item is valid on its own.
  for (int i = 0; i < stack_->count(); ++i) {
    const auto cur_widget = qt::qPointerCast<BaseSettingWidget>(stack_->widget(i));
    if (!cur_widget->isValid()) {
      setCurrentPage(cur_widget);
      return false;
    }
  }

  switch (propulsion_system->type()) {
    case PropulsionSystem::kElectric: {
      // Confirm that DShot channels are set for electric motors.
      for (const auto& elem : uadf_.thrusts) {
        const auto joint_name = QString::fromStdString(elem.first);
        if (!hardware->dshot()->contains(joint_name)) {
          qt::qWarnBox(this, "Please specify a DShot channel for electric rotor \"" + joint_name + "\".");
          setCurrentPage(hardware);
          return false;
        }
      }

      break;
    }
    case PropulsionSystem::kIce: {
      break;
    }
    default: {
      throw;
    }
  }

  return true;
}

YAML::Node SettingsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < stack_->count(); ++i) {
    const auto page = qt::qConstPointerCast<BaseSettingWidget>(stack_->widget(i));
    node[page->name()] = page->dump();
  }

  return node;
}

bool SettingsWidget::load(const YAML::Node& node)
{
  bool success = true;

  for (int i = 0; i < stack_->count(); ++i) {
    const auto page = qt::qPointerCast<BaseSettingWidget>(stack_->widget(i));
    try {
      page->load(node[page->name()]);
    }
    catch (const std::exception& e) {
      qt::qErrorBox(this, "Failed to load settings of \"" + QString(page->name()) + "\":\n\n" + e.what());
      success = false;
    }
  }

  return success;
}

void SettingsWidget::setFrameType(FrameType type)
{
  // Disable controller settings if the frame type is undefined.
  if (type == FrameType::kUndefined) {
    setPageEnabled(controller, false);
    setPageEnabled(mission, false);
  }

  controller->setFrameType(type);
  mission->setFrameType(type);
}

int SettingsWidget::getIndex(BaseSettingWidget* page) const
{
  const auto idx = stack_->indexOf(page);
  TOBAS_CHECK(idx >= 0);
  return idx;
}

void SettingsWidget::addPage(BaseSettingWidget* page)
{
  const auto idx = stack_->addWidget(page);
  navigation_->addEntry(page->name(), idx);
}

void SettingsWidget::setCurrentPage(int idx)
{
  navigation_->setCurrentEntry(idx);
}

void SettingsWidget::setCurrentPage(BaseSettingWidget* page)
{
  setCurrentPage(getIndex(page));
}

void SettingsWidget::setPageEnabled(int idx, bool enabled)
{
  stack_->widget(idx)->setEnabled(enabled);
  navigation_->setEntryEnabled(idx, enabled);
}

void SettingsWidget::setPageEnabled(BaseSettingWidget* page, bool enabled)
{
  setPageEnabled(getIndex(page), enabled);
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
