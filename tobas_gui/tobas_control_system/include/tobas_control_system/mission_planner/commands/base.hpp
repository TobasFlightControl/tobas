#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "../command.hpp"
#include "../fields/base.hpp"

namespace gui
{
namespace gcs
{
struct BaseCommandData
{
  using SharedPtr = std::shared_ptr<BaseCommandData>;

  virtual command_t type() const = 0;
};

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
  virtual BaseCommandData::SharedPtr data() const = 0;

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
}  // namespace gcs
}  // namespace gui
