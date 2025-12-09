#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_wpa_supplicant/export.hpp>
#include <tobas_wpa_supplicant/parse.hpp>

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

  static constexpr int kKeyMgmtCol = 0;
  static constexpr int kSsidCol = 1;
  static constexpr int kPskCol = 2;
  static constexpr int kPriorityCol = 3;
  static constexpr int kHiddenCol = 4;
  static constexpr int kNumCols = 5;

public:
  explicit WifiClientWidget();

  const char* name() const override;
  const char* title() const override;

  void reset() override;

private:
  wpa::Data wpa_data_;
  wpa::Parser wpa_parser_;
  wpa::Exporter wpa_exporter_;

  QPushButton* read_button_;
  QPushButton* add_button_;
  QPushButton* remove_button_;
  QPushButton* clear_button_;

  qt::TableWidget* table_;

  QString getKeyMgmt(int row) const;
  QString getSsid(int row) const;
  QString getPsk(int row) const;
  int getPriority(int row) const;
  bool getHidden(int row) const;

  void addRow(const QString& key_mgmt, const QString& ssid, const QString& psk, int priority, bool hidden);
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
