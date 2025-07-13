#include "tobas_setup_assistant/setting_tabs/hardware/dshot.hpp"

#include <ranges>

#include <QDebug>
#include <QHeaderView>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/hardware/constants.hpp"

namespace gui
{
namespace sa
{
namespace hw
{
DShotWidget::DShotWidget(const RobotInfo& robot, const Signals& sig) : super(0, kNumCols), robot_(robot)
{
  horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  horizontalHeader()->setMinimumSectionSize(kTableHeaderSectionSize);
  setHorizontalHeaderLabels({ kTargetNameLabel, kBidirectionalLabel });

  connect(&sig, &Signals::propulsionTypeChanged, this, &self::onPropulsionTypeChanged);
}

void DShotWidget::updateInternalDataStructures()
{
  // 現在の行数を保存
  const auto rows = rowCount();

  // 設定をリセットするために一旦全削除
  removeAll();

  // 更新された選択肢でチャンネルを加え直す
  for (int _ = 0; _ < rows; ++_) {
    addLastChannel();
  }
}

YAML::Node DShotWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Sequence);

  for (int channel = 0; channel < rowCount(); ++channel) {
    YAML::Node sub_node(YAML::NodeType::Map);

    sub_node[kTargetNameLabel] = target_names_.at(channel)->currentText();
    sub_node[kBidirectionalLabel] = bidirectional_.at(channel)->isChecked();

    node.push_back(sub_node);
  }

  return node;
}

void DShotWidget::load(const YAML::Node& node)
{
  for (const auto& [channel, sub_node] : std::views::enumerate(node)) {
    target_names_.at(channel)->setCurrentText(sub_node[kTargetNameLabel].as<QString>());
    bidirectional_.at(channel)->setChecked(sub_node[kBidirectionalLabel].as<bool>());
  }
}

void DShotWidget::setNumChannels(int num)
{
  while (true) {
    if (num > rowCount()) {
      addLastChannel();
    }
    else if (num < rowCount()) {
      removeLastChannel();
    }
    else {
      return;
    }
  }
}

QString DShotWidget::targetName(int channel) const
{
  return target_names_.at(channel)->currentText();
}

bool DShotWidget::bidirectional(int channel) const
{
  return bidirectional_.at(channel)->isChecked();
}

bool DShotWidget::contains(const QString& target_name) const
{
  for (int channel = 0; channel < rowCount(); ++channel) {
    if (targetName(channel) == target_name) {
      return true;
    }
  }

  return false;
}

int DShotWidget::channel(const QString& target_name) const
{
  for (int channel = 0; channel < rowCount(); ++channel) {
    if (targetName(channel) == target_name) {
      return channel;
    }
  }

  qWarning() << "Failed to find \"" << target_name << "\".";
  return -1;
}

void DShotWidget::addLastChannel()
{
  const auto row = rowCount();

  // Target name
  const auto target_name = new qt::ComboBox();
  target_name->addItem("");  // 未選択
  switch (prop_type_) {
    case tobas::propulsion_system_t::ELECTRIC: {
      for (const auto& [joint_name, _] : robot_.uadf().thrusts) {
        target_name->addItem(QString::fromStdString(joint_name));
      }
      break;
    }
    case tobas::propulsion_system_t::ICE: {
      break;
    }
    default:
      throw;
  }

  // Bidirectional
  const auto bidirectional = new QPushButton();
  bidirectional->setCheckable(true);
  setBidirectionalButtonText(bidirectional, true);  // デフォルトで双方向通信
  connect(
    bidirectional,
    &QPushButton::toggled,
    std::bind(&self::onBidirectionalButtonToggled, this, bidirectional, std::placeholders::_1));
  bidirectional->setEnabled(false);  // TODO: 単方向にも対応

  // Insert table row
  insertRow(row);
  setCellWidget(row, kTargetNameCol, target_name);
  setCellWidget(row, kBidirectionalCol, bidirectional);

  // Save each field
  target_names_.append(target_name);
  bidirectional_.append(bidirectional);
}

void DShotWidget::removeLastChannel()
{
  const auto row = rowCount() - 1;

  if (row < 0) {
    return;
  }

  const auto target_name = targetName(row);

  removeRow(row);

  if (!target_name.isEmpty()) {
    qt::qWarnBox(this, "PWM configuration for \"" + target_name + "\" has been removed.");
  }
}

void DShotWidget::setBidirectionalButtonText(QPushButton* button, bool checked)
{
  if (checked) {
    button->setText("Enabled");
  }
  else {
    button->setText("Disabled");
  }
}

void DShotWidget::onPropulsionTypeChanged(const tobas::propulsion_system_t& new_prop_type)
{
  if (new_prop_type == prop_type_) {
    return;
  }

  // 前の推進系の不要な選択肢を外す
  switch (prop_type_) {
    case tobas::propulsion_system_t::ELECTRIC: {
      for (const auto& target_name : target_names_) {
        for (const auto& [joint_name, _] : robot_.uadf().thrusts) {
          if (target_name->currentText().toStdString() == joint_name) {
            target_name->setCurrentText("");
          }
          target_name->removeText(QString::fromStdString(joint_name));
        }
      }
      break;
    }
    case tobas::propulsion_system_t::ICE: {
      break;
    }
    default:
      throw;
  }

  // 新しい推進系の選択肢を追加
  switch (prop_type_) {
    case tobas::propulsion_system_t::ELECTRIC: {
      for (const auto& target_name : target_names_) {
        for (const auto& [joint_name, _] : robot_.uadf().thrusts) {
          target_name->addItem(QString::fromStdString(joint_name));
        }
      }
      break;
    }
    case tobas::propulsion_system_t::ICE: {
      break;
    }
    default:
      throw;
  }

  prop_type_ = new_prop_type;
}

void DShotWidget::onBidirectionalButtonToggled(QPushButton* button, bool checked)
{
  setBidirectionalButtonText(button, checked);
}
}  // namespace hw
}  // namespace sa
}  // namespace gui
