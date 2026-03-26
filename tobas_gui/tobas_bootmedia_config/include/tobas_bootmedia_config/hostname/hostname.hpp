#pragma once

#include <QLineEdit>
#include <QPushButton>

#include <tobas_qt_tools/widgets/label.hpp>

#include "../base.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class HostnameWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = HostnameWidget;
  using super = BaseConfigWidget;

public:
  explicit HostnameWidget();

  const char* name() const override;
  const char* title() const override;

  void reset() override;

private:
  QPushButton* read_button_;
  QPushButton* write_button_;

  QLineEdit* hostname_;
  qt::Label* warn_text_;

  QString getHostname() const;

  bool writeHostnameFile(const QString& hostname);
  bool writeHostsFile(const QString& hostname);

  static std::string hostnameFilePath();
  static std::string hostsFilePath();

private Q_SLOTS:
  void onHostnameChanged(const QString& hostname);
  void onReadButtonClicked();
  void onWriteButtonClicked();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
