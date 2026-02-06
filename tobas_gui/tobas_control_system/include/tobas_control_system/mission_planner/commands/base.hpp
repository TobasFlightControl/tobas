#pragma once

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "../fields/base.hpp"

namespace gui
{
namespace ctrl
{
class BaseCommandWidget : public QWidget
{
  Q_OBJECT

  using self = BaseCommandWidget;
  using super = QWidget;

  static constexpr int kLablePSize = 12;
  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

Q_SIGNALS:
  void updated();
  void deleteButtonClicked();

public:
  explicit BaseCommandWidget();

  virtual const char* name() const = 0;

protected:
  void addField(field::BaseField* field);

private:
  QLabel* label_;
  QPushButton* delete_button_;
  qt::FormLayout* form_;

private Q_SLOTS:
  void initialize();
  void onFieldUpdated();
  void onDeleteButtonClicked();
};
}  // namespace ctrl
}  // namespace gui
