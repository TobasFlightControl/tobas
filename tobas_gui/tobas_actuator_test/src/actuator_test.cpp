#include "tobas_actuator_test/actuator_test.hpp"

#include <QApplication>
#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>

namespace gui
{
namespace at
{
ActuatorTestWidget::ActuatorTestWidget(
  rclcpp::Node::SharedPtr node,
  const RosQtBridge& bridge,
  const kdl::Tree& tree,
  const tobas::Drone& drone)
  : drone_(drone)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  tabs_ = new qt::VerticalTabWidget();
  tabs_->enableWheelEvent(false);
  rows->addWidget(tabs_);

  rotor_test_ = new RotorTestWidget(node, bridge, drone);
  joint_test_ = new JointTestWidget(node, bridge, tree, drone);

  tabs_->addTab(rotor_test_, rotor_test_->name());
  tabs_->addTab(joint_test_, joint_test_->name());

  tabs_->setTabSize(kTabWidth, kTabHeight);

  // プロジェクトが読み込まれるまでは無効化
  setTabsEnabled(false);
}

void ActuatorTestWidget::reset()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = qt::qPointerCast<BaseWidget>(tabs_->widget(i));
    widget->reset();
  }
}

void ActuatorTestWidget::updateInternalDataStructures()
{
  reset();

  rotor_test_->updateInternalDataStructures();
  joint_test_->updateInternalDataStructures();

  // テスト系は1つ以上のチャンネルが登録されているときのみ有効化
  tabs_->setTabEnabled(rotor_test_, rotor_test_->numRegisteredChannels() > 0);
  tabs_->setTabEnabled(joint_test_, joint_test_->numRegisteredChannels() > 0);

  // 各タブを有効化
  setTabsEnabled(true);

  // タブを表示・非表示した際の歪みを整える
  tabs_->update();
}

void ActuatorTestWidget::setTabsEnabled(bool enabled)
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = qt::qPointerCast<BaseWidget>(tabs_->widget(i));
    widget->setEnabled(enabled);
  }
}
}  // namespace at
}  // namespace gui
