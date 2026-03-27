#include "tobas_actuator_test/actuator_test.hpp"

#include <tobas_qt_tools/cast.hpp>

namespace tobas
{
namespace gui
{
namespace at
{
ActuatorTestWidget::ActuatorTestWidget(
  rclcpp::Node::SharedPtr node,
  const RosQtBridge& bridge,
  const kdl::Tree& tree,
  const Drone& drone)
  : drone_(drone)
{
  setTabSize(kTabWidth, kTabHeight);
  enableWheelEvent(false);

  rotor_test_ = new RotorTestWidget(node, bridge, drone);
  addTab(rotor_test_, "Rotor Test");

  joint_test_ = new JointTestWidget(node, bridge, tree, drone);
  addTab(joint_test_, "Joint Test");

  setTabsEnabled(false);
}

void ActuatorTestWidget::reset()
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->reset();
  }
}

void ActuatorTestWidget::updateInternalDataStructures()
{
  reset();

  rotor_test_->updateInternalDataStructures();
  joint_test_->updateInternalDataStructures();

  // テスト系は1つ以上のチャンネルが登録されているときのみ有効化
  setTabEnabled(rotor_test_, rotor_test_->numRegisteredChannels() > 0);
  setTabEnabled(joint_test_, joint_test_->numRegisteredChannels() > 0);

  // 各タブを有効化
  setTabsEnabled(true);

  // タブを表示・非表示した際の歪みを整える
  update();
}

BaseWidget* ActuatorTestWidget::getWidget(int index)
{
  return qt::qPointerCast<BaseWidget>(widget(index));
}

const BaseWidget* ActuatorTestWidget::getWidget(int index) const
{
  return qt::qConstPointerCast<BaseWidget>(widget(index));
}

void ActuatorTestWidget::setTabsEnabled(bool enabled)
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->setEnabled(enabled);
  }
}
}  // namespace at
}  // namespace gui
}  // namespace tobas
