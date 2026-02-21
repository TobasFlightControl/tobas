#include "tobas_setup_assistant/setting_tabs/network.hpp"

#include <ranges>

#include <QDebug>
#include <QHBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
NetworkWidget::NetworkWidget()
{
  nif_btn_group_ = new QButtonGroup(this);
  nif_btn_group_->setExclusive(true);

  int id = 0;
  const auto auto_btn = addNifTypeButton("Auto", id++);
  const auto wired_btn = addNifTypeButton("Wired (Ethernet)", id++);
  const auto wireless_btn = addNifTypeButton("Wireless (Wi-Fi Client)", id++);
  const auto ap_btn = addNifTypeButton("Access Point (Wi-Fi Hotspot)", id++);
  const auto other_btn = addNifTypeButton("Other", id++);

  nif_btn_group_->button(kAutoIdx)->setChecked(true);  // Default

  other_nif_name_ = new QLineEdit();
  other_nif_name_->setPlaceholderText("e.g. wwan0, eth1, enx...");
  other_nif_name_->setEnabled(false);

  const auto other_row = new QHBoxLayout();
  other_row->addWidget(other_btn);
  other_row->addWidget(other_nif_name_);

  addWidget(new qt::Label("Network Interface", cmn::kLabelPSize, QFont::Bold));
  addWidget(auto_btn);
  addWidget(wired_btn);
  addWidget(wireless_btn);
  addWidget(ap_btn);
  addLayout(other_row);

  addStretch();

  connect(other_btn, &QRadioButton::toggled, this, &self::onOtherButtonToggled);
}

const char* NetworkWidget::name() const
{
  return "Network";
}

const char* NetworkWidget::title() const
{
  return "Configure Network";
}

const char* NetworkWidget::description() const
{
  return "Configure the network settings used by the Flight Controller (FC). "
         "All devices that communicate with the FC via ROS must match the settings specified here.";
}

void NetworkWidget::updateInternalDataStructures()
{
  return;
}

bool NetworkWidget::isValid()
{
  if (nif_btn_group_->checkedId() == kOtherIdx) {
    if (other_nif_name_->text().isEmpty()) {
      qt::qWarnBox(this, "Please specify a network interface name.");
      return false;
    }
  }

  return true;
}

YAML::Node NetworkWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  const auto cur_btn = nif_btn_group_->checkedButton();
  node[kNifTypeKey] = cur_btn->text();

  node[kOtherNifNameKey] = other_nif_name_->text();

  return node;
}

void NetworkWidget::load(const YAML::Node& node)
{
  const auto nif_type_text = node[kNifTypeKey].as<QString>();
  for (const auto& [idx, btn] : std::views::enumerate(nif_btn_group_->buttons())) {
    if (btn->text() == nif_type_text) {
      nif_btn_group_->button(idx)->setChecked(true);
      break;
    }
  }

  other_nif_name_->setText(node[kOtherNifNameKey].as<QString>());
}

QString NetworkWidget::networkInterfaceName() const
{
  switch (nif_btn_group_->checkedId()) {
    case kAutoIdx:
      return {};
    case kWiredIdx:
      return "eth0";
    case kWirelessIdx:
      return "wlan0";
    case kAccessPointIdx:
      return "ap0";
    case kOtherIdx:
      return other_nif_name_->text();
    default:
      qWarning() << "Invalid network interface.";
      return {};
  }
}

QRadioButton* NetworkWidget::addNifTypeButton(const QString& text, int id)
{
  const auto btn = new QRadioButton(text);
  nif_btn_group_->addButton(btn, id);
  return btn;
}

void NetworkWidget::onOtherButtonToggled(bool checked)
{
  other_nif_name_->setEnabled(checked);

  if (checked) {
    other_nif_name_->setFocus();
    other_nif_name_->selectAll();
  }
}
}  // namespace sa
}  // namespace gui
