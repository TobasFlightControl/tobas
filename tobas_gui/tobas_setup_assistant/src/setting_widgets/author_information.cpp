// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/author_information.hpp"

#include <tobas_git/core.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace
{
// Keep the existing serialized keys independent of the form labels.
constexpr char kNameKey[] = "Name of the Maintainer";
constexpr char kEmailKey[] = "Email of the Maintainer";
}  // namespace

AuthorInformationWidget::AuthorInformationWidget()
{
  const auto form = new qt::FormLayout();
  addLayout(form);

  name_ = new QLineEdit();
  name_->setText(getDefaultName());
  form->addRow(kNameKey, name_);
  form->labelForField(name_)->setToolTip(name_->toolTip());

  email_ = new QLineEdit();
  email_->setText(getDefaultEmail());
  form->addRow(kEmailKey, email_);
  form->labelForField(email_)->setToolTip(email_->toolTip());

  addStretch();
}

const char* AuthorInformationWidget::name() const
{
  return "Author Information";
}

const char* AuthorInformationWidget::title() const
{
  return "Specify Author Information";
}

const char* AuthorInformationWidget::description() const
{
  return "Enter the name and email address of the person administering the Tobas project "
         "that you are creating with the Setup Assistant. "
         "This step is important for keeping track of package ownership and for any necessary future communications.";
}

void AuthorInformationWidget::updateInternalDataStructures()
{
  return;
}

bool AuthorInformationWidget::isValid()
{
  const auto author_name = name_->text();
  if (author_name.isEmpty()) {
    qt::qWarnBox(this, "Please specify author name.");
    return false;
  }

  const auto author_email = email_->text();
  if (author_email.isEmpty()) {
    qt::qWarnBox(this, "Please specify author email address.");
    return false;
  }
  if (!str::isValidEmail(author_email.toStdString())) {
    qt::qWarnBox(this, "Invalid email address.");
    return false;
  }

  return true;
}

YAML::Node AuthorInformationWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNameKey] = name_->text();
  node[kEmailKey] = email_->text();

  return node;
}

void AuthorInformationWidget::load(const YAML::Node& node)
{
  name_->setText(node[kNameKey].as<QString>());
  email_->setText(node[kEmailKey].as<QString>());
}

QString AuthorInformationWidget::authorName() const
{
  return name_->text();
}

QString AuthorInformationWidget::authorEmail() const
{
  return email_->text();
}

QString AuthorInformationWidget::getDefaultName()
{
  const auto git_user_name = git::getGitConfigValue("user.name");
  if (!git_user_name.empty()) {
    return QString::fromStdString(git_user_name);
  }

  const auto user_name = ros2::getUserName();
  if (user_name) {
    return QString(user_name);
  }

  return QString("todo");
}

QString AuthorInformationWidget::getDefaultEmail()
{
  const auto git_user_email = git::getGitConfigValue("user.email");
  if (!git_user_email.empty()) {
    return QString::fromStdString(git_user_email);
  }

  return QString("todo@todo.todo");
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
