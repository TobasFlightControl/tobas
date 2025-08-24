#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_wpa_supplicant_parser/parser.hpp>

#include "../base.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class WifiClientWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = WifiClientWidget;
  using super = BaseConfigWidget;

  static constexpr int kColWidth = 300;

  static constexpr int kSsidCol = 0;
  static constexpr int kPskCol = 1;
  static constexpr int kPriorityCol = 2;
  static constexpr int kNumCols = 3;

public:
  explicit WifiClientWidget();

  const char* name() const override;
  const char* title() const override;

  void reset() override;

private:
  wpa::WpaSupplicantParser wpa_parser_;

  QPushButton* read_button_;
  QPushButton* add_button_;
  QPushButton* remove_button_;
  QPushButton* clear_button_;

  qt::TableWidget* table_;

  QString getSsid(int row) const;
  QString getPsk(int row) const;
  int getPriority(int row) const;

  void addRow(const QString& ssid, const QString& psk, int priority);
  bool writeCurrentConfig();

  static std::string configPath();

private Q_SLOTS:
  void onReadButtonClicked();
  void onAddButtonClicked();
  void onRemoveButtonClicked();
  void onClearButtonClicked();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
