#pragma once

#include <QPushButton>

#include <wpa_supplicant_parser/parser.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

#include "../base.hpp"
#include "./read_thread.hpp"
#include "./write_thread.hpp"

namespace gui
{
namespace hw
{
class NetworkSettingWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = NetworkSettingWidget;
  using super = BaseHardwareSetupWidget;

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

private:
  wpa::WPASupplicantParser wpa_parser_;

  QPushButton* read_button_;
  QPushButton* write_button_;
  QPushButton* add_button_;
  QPushButton* remove_button_;

  qt::TableWidget* table_;

  qt::WaitSpinnerWidget spinner_;

  ReadWPASupplicantThread read_thread_;
  WriteWPASupplicantThread write_thread_;

  void addRow(const std::string& ssid, const std::string& psk);

private Q_SLOTS:
  void onReadButtonClicked();
  void onWriteButtonClicked();
  void onAddButtonClicked();
  void onRemoveButtonClicked();

  void onReadThreadFinished(bool success, const QString& message);
  void onWriteThreadFinished(bool success, const QString& message);
};
}  // namespace hw
}  // namespace gui
