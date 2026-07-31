// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/author_information.hpp"

#include <tobas_git/core.hpp>
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
QString getDefaultName()
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

QString getDefaultEmail()
{
  const auto git_user_email = git::getGitConfigValue("user.email");
  if (!git_user_email.empty()) {
    return QString::fromStdString(git_user_email);
  }

  return QString("todo@todo.todo");
}
}  // namespace

AuthorInformationWidget::AuthorInformationWidget()
{
  name_ = new ParamGetterWidget_LineEdit("Name of the Maintainer", "");
  addWidget(name_);

  email_ = new ParamGetterWidget_LineEdit("Email of the Maintainer", "");
  addWidget(email_);

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

void AuthorInformationWidget::setToDefaults()
{
  name_->setValue(getDefaultName());
  email_->setValue(getDefaultEmail());
}

bool AuthorInformationWidget::isValid()
{
  const auto author_name = name_->getValue();
  if (author_name.isEmpty()) {
    qt::qWarnBox(this, "Please specify author name.");
    return false;
  }

  const auto author_email = email_->getValue();
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

  node[name_->name()] = name_->getValue();
  node[email_->name()] = email_->getValue();

  return node;
}

void AuthorInformationWidget::load(const YAML::Node& node)
{
  name_->setValue(node[name_->name()].as<QString>());
  email_->setValue(node[email_->name()].as<QString>());
}

QString AuthorInformationWidget::authorName() const
{
  return name_->getValue();
}

QString AuthorInformationWidget::authorEmail() const
{
  return email_->getValue();
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
