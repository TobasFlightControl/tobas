#include "tobas_setup_assistant/setting_tabs/propulsion_system/propulsion_system.hpp"

#include <ranges>

#include <QDebug>
#include <QRadioButton>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
PropulsionSystemWidget::PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf, Signals& sig)
  : sig_(sig)
{
  type_btn_group_ = new QButtonGroup(this);
  type_btn_group_->setExclusive(true);

  propulsion_stack_ = new qt::StackedWidget();

  const auto eprop = new electric::PropulsionSystemWidget(node, uadf);
  const auto eprop_ckb = new QRadioButton(eprop->name());
  type_btn_group_->addButton(eprop_ckb);
  type_btn_group_->setId(eprop_ckb, kElectricId);
  propulsion_stack_->addWidget(eprop);

  const auto iprop = new ice::PropulsionSystemWidget(node, uadf);
  const auto iprop_ckb = new QRadioButton(iprop->name());
  type_btn_group_->addButton(iprop_ckb);
  type_btn_group_->setId(iprop_ckb, kIceId);
  propulsion_stack_->addWidget(iprop);

  // デフォルト
  setCurrentIndex(0);

  // Layout
  addWidget(eprop_ckb);
  addWidget(iprop_ckb);
  addSpacing(50);
  addWidget(propulsion_stack_);

  // Connection
  connect(type_btn_group_, &QButtonGroup::idClicked, this, &self::onPropulsionTypeClicked);
}

const char* PropulsionSystemWidget::name() const
{
  return "Propulsion System";
}

const char* PropulsionSystemWidget::title() const
{
  return "Define Propulsion System";
}

const char* PropulsionSystemWidget::description() const
{
  return "Build the mathematical model for your propulsion system. "
         "Tobas supports two configurations:\n"
         "  1. Electric – battery‑powered with fixed‑pitch propellers\n"
         "  2. ICE – an internal‑combustion engine driving variable‑pitch propellers through gearboxes\n"
         "An accurate propulsion model is critical to maximizing aircraft performance. "
         "Select the appropriate architecture and enter the required parameters for each field.";
}

void PropulsionSystemWidget::updateInternalDataStructures()
{
  for (int i = 0; i < propulsion_stack_->count(); ++i) {
    const auto propulsion = widget(i);
    propulsion->updateInternalDataStructures();
  }
}

bool PropulsionSystemWidget::isValid()
{
  if (!selected()->isValid()) {
    return false;
  }

  return true;
}

YAML::Node PropulsionSystemWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  const auto type_btn = type_btn_group_->checkedButton();
  node[kTypeKey] = type_btn->text();

  for (int i = 0; i < propulsion_stack_->count(); ++i) {
    const auto propulsion = widget(i);
    node[propulsion->name()] = propulsion->dump();
  }

  return node;
}

void PropulsionSystemWidget::load(const YAML::Node& node)
{
  // 実際にユーザが操作するときと同じように推進系の型の選択と変更の通知を行う
  const auto type_text = node[kTypeKey].as<QString>();
  for (const auto& [idx, button] : std::views::enumerate(type_btn_group_->buttons())) {
    if (button->text() == type_text) {
      setCurrentIndex(idx);
      Q_EMIT sig_.propulsionTypeChanged(widget(idx)->type());
      break;
    }
  }

  for (int i = 0; i < propulsion_stack_->count(); ++i) {
    const auto propulsion = widget(i);
    propulsion->load(node[propulsion->name()]);
  }
}

tobas::PropulsionSystem PropulsionSystemWidget::type() const
{
  return selected()->type();
}

int PropulsionSystemWidget::numUnits() const
{
  return selected()->numUnits();
}

QString PropulsionSystemWidget::linkName(int index) const
{
  return selected()->linkName(index);
}

BasePropulsionSystemWidget* PropulsionSystemWidget::widget(int index)
{
  return qt::qPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->widget(index));
}

const BasePropulsionSystemWidget* PropulsionSystemWidget::widget(int index) const
{
  return qt::qConstPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->widget(index));
}

BasePropulsionSystemWidget* PropulsionSystemWidget::selected()
{
  return qt::qPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->currentWidget());
}

const BasePropulsionSystemWidget* PropulsionSystemWidget::selected() const
{
  return qt::qConstPointerCast<BasePropulsionSystemWidget>(propulsion_stack_->currentWidget());
}

void PropulsionSystemWidget::setCurrentButtonIndex(int index)
{
  // チェックされているボタンが切り替わらないなら何もしない
  if (type_btn_group_->checkedId() == index) {
    return;
  }

  // 切り替え前後のボタンを取得
  const auto old_btn = type_btn_group_->checkedButton();
  const auto new_btn = type_btn_group_->button(index);

  // 全てのシグナルをブロック (nullptrを渡しても問題ない)
  const QSignalBlocker block_group(type_btn_group_);
  const QSignalBlocker block_old_btn(old_btn);
  const QSignalBlocker block_new_btn(new_btn);

  // 新しいボタンにチェック (exclusiveなのでold_btnは自動的にチェックが外れる)
  new_btn->setChecked(true);
}

void PropulsionSystemWidget::setCurrentIndex(int index)
{
  setCurrentButtonIndex(index);
  propulsion_stack_->setCurrentIndex(index);
  cur_idx_ = index;
}

void PropulsionSystemWidget::onPropulsionTypeClicked(int new_idx)
{
  qDebug() << "PropulsionSystemWidget::onPropulsionTypeChanged(" << new_idx << ")";

  if (new_idx == cur_idx_) {
    return;
  }

  if (!qt::yesOrNo(
        this, "Changing the propulsion type will reset the wiring settings. Do you want to continue?", qt::WARN)) {
    setCurrentButtonIndex(cur_idx_);
    return;
  }

  // 推進系のウィジェットを切り替える
  propulsion_stack_->setCurrentIndex(new_idx);
  cur_idx_ = new_idx;

  // 推進系の型が変わったことを他のウィジェットに通知
  Q_EMIT sig_.propulsionTypeChanged(widget(new_idx)->type());
}
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
