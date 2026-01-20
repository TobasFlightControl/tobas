#include "tobas_setup_assistant/setting_tabs/network.hpp"

#include <QDebug>
#include <QFormLayout>

#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
NetworkWidget::NetworkWidget()
{
  nif_type_ = new ParamGetterWidget_ComboBox("Network Interface");
  nif_type_->addChoices(
    { "Auto", "Wired (Ethernet)", "Wireless (Wi-Fi Client)", "Access Point (Wi-Fi Hotspot)", "Other" });

  nif_name_label_ = new QLabel(kNifNameLabel);
  nif_name_ = new QLineEdit();

  nif_name_label_->setVisible(false);
  nif_name_->setVisible(false);

  const auto form = new QFormLayout();
  form->addRow(nif_name_label_, nif_name_);

  addWidget(nif_type_);
  addLayout(form);

  addStretch();

  connect(nif_type_, &ParamGetterWidget_ComboBox::indexChanged, this, &self::onNifTypeChanged);
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
  return true;
}

YAML::Node NetworkWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[nif_type_->name()] = nif_type_->getValue();
  node[kNifNameLabel] = nif_name_->text();

  return node;
}

void NetworkWidget::load(const YAML::Node& node)
{
  nif_type_->setValue(node[nif_type_->name()].as<QString>());
  nif_name_->setText(node[kNifNameLabel].as<QString>());
}

QString NetworkWidget::networkInterfaceName() const
{
  const auto idx = nif_type_->currentIndex();

  switch (idx) {
    case kAutoIdx:
      return {};
    case kWiredIdx:
      return "end0";
    case kWirelessIdx:
      return "wlan0";
    case kAccessPointIdx:
      return "ap0";
    case kOtherIdx:
      return nif_name_->text();
    default:
      qWarning() << "Invalid network interface index: " << idx;
      return {};
  }
}

void NetworkWidget::onNifTypeChanged(int index)
{
  const auto show = index == kOtherIdx;
  nif_name_label_->setVisible(show);
  nif_name_->setVisible(show);
}
}  // namespace sa
}  // namespace gui
