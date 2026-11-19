// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/maintainer_information.hpp"

#include <QDebug>

#include <tobas_git/core.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas::gui::sa
{
namespace
{
constexpr char kNameLabel[] = "Maintainer Name";
constexpr char kEmailLabel[] = "Maintainer Email";

QString getDefaultName()
{
  const auto git_user_name = git::getGitConfigValue("user.name");
  if (!git_user_name) {
    qWarning() << "Failed to get the git user name:" << QString::fromStdString(git_user_name.error());
  }
  else if (!git_user_name->empty()) {
    return QString::fromStdString(*git_user_name);
  }
  else {
    qWarning() << "Git user name is not set.";
  }

  const auto user_name = ros2::getUserName();
  if (user_name) {
    return QString(*user_name);
  }
  else {
    qWarning() << "Failed to get the user name:" << QString::fromStdString(user_name.error());
  }

  return QString("todo");
}

QString getDefaultEmail()
{
  const auto git_user_email = git::getGitConfigValue("user.email");
  if (!git_user_email) {
    qWarning() << "Failed to get the git user email:" << QString::fromStdString(git_user_email.error());
  }
  else if (!git_user_email->empty()) {
    return QString::fromStdString(*git_user_email);
  }
  else {
    qWarning() << "Git user email is not set.";
  }

  return QString("todo@todo.todo");
}
}  // namespace

MaintainerInformationWidget::MaintainerInformationWidget()
{
  const auto form = new qt::FormLayout(this);

  name_ = new QLineEdit();
  name_->setToolTip("Name of the maintainer recorded in the generated ROS package manifests.");
  form->addRow(kNameLabel, name_);
  form->labelForField(name_)->setToolTip(name_->toolTip());

  email_ = new QLineEdit();
  email_->setToolTip(
    "Email address of the maintainer recorded in the generated ROS package manifests. Enter a valid email address.");
  form->addRow(kEmailLabel, email_);
  form->labelForField(email_)->setToolTip(email_->toolTip());
}

const char* MaintainerInformationWidget::name() const
{
  return "Maintainer Information";
}

const char* MaintainerInformationWidget::title() const
{
  return "Enter Maintainer Information";
}

const char* MaintainerInformationWidget::description() const
{
  return "Enter the maintainer name and email address to include in the generated ROS package manifests.";
}

void MaintainerInformationWidget::updateInternalDataStructures()
{
  return;
}

void MaintainerInformationWidget::setToDefaults()
{
  name_->setText(getDefaultName());
  email_->setText(getDefaultEmail());
}

bool MaintainerInformationWidget::isValid()
{
  const auto maintainer_name = name_->text();
  if (maintainer_name.isEmpty()) {
    qt::qWarnBox(this, "Please enter the maintainer's name.");
    return false;
  }

  const auto maintainer_email = email_->text();
  if (maintainer_email.isEmpty()) {
    qt::qWarnBox(this, "Please enter the maintainer's email address.");
    return false;
  }
  if (!str::isValidEmail(maintainer_email.toStdString())) {
    qt::qWarnBox(this, "Invalid email address.");
    return false;
  }

  return true;
}

YAML::Node MaintainerInformationWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNameLabel] = name_->text();
  node[kEmailLabel] = email_->text();

  return node;
}

void MaintainerInformationWidget::load(const YAML::Node& _node)
{
  name_->setText(_node[kNameLabel].as<QString>());
  email_->setText(_node[kEmailLabel].as<QString>());
}

QString MaintainerInformationWidget::maintainerName() const
{
  return name_->text();
}

QString MaintainerInformationWidget::maintainerEmail() const
{
  return email_->text();
}
}  // namespace tobas::gui::sa
