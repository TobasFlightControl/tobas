#pragma once

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>

#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_setup_assistant/constants.hpp"

namespace gui
{
namespace sa
{
/* A base class of a widget for getting a user parameter. */
template <typename T>
class ParamGetterWidget : public QWidget
{
  using super = QWidget;

public:
  explicit ParamGetterWidget(const QString& param_name, const QString& description_text);

  virtual T getValue() const = 0;
  virtual bool setValue(const T& src) = 0;

  std::string name() const;

protected:
  QVBoxLayout* rows_;
  QLabel* label_;
};

template <typename T>
ParamGetterWidget<T>::ParamGetterWidget(const QString& param_name, const QString& description_text)
{
  rows_ = new QVBoxLayout();
  setLayout(rows_);

  label_ = new QLabel(param_name);
  label_->setFont(qt::DefaultFont(kLabelPSize, QFont::Bold));
  label_->setAlignment(Qt::AlignTop);
  rows_->addWidget(label_);

  if (!description_text.isEmpty())
  {
    const auto description = new qt::DescriptionWidget(description_text, kBodyPSize);
    rows_->addWidget(description);
  }
}

template <typename T>
std::string ParamGetterWidget<T>::name() const
{
  return label_->text().toStdString();
}
}  // namespace sa
}  // namespace gui
