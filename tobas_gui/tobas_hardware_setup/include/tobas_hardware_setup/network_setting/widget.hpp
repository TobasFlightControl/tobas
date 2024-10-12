#pragma once

#include <QPushButton>

#include <wpa_supplicant_parser/parser.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>

#include "../base.hpp"

namespace gui
{
namespace hardware_setup
{
class NetworkSettingWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = NetworkSettingWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr char kFilePath[] = "/etc/wpa_supplicant/wpa_supplicant.conf";

  static constexpr int kColWidth = 200;
  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

  static constexpr int kSSIDCol = 0;
  static constexpr int kPSKCol = 1;
  static constexpr int kNumCols = 2;

public:
  explicit NetworkSettingWidget(rclcpp::Node::SharedPtr node);

  const char* name() const override;
  const char* title() const override;

  void onInit() override;

private:
  wpa::WPASupplicantParser wpa_parser_;
  ssh::SSHClient ssh_client_;

  QPushButton* read_button_;
  QPushButton* write_button_;
  QPushButton* add_button_;
  QPushButton* remove_button_;

  qt::TableWidget* table_;

  void addRow(const std::string& ssid, const std::string& psk);

private Q_SLOTS:
  void onReadButtonClicked();
  void onWriteButtonClicked();
  void onAddButtonClicked();
  void onRemoveButtonClicked();
};
}  // namespace hardware_setup
}  // namespace gui
