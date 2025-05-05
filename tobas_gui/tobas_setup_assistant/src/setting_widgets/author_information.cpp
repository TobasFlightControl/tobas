#include "tobas_setup_assistant/setting_tabs/author_information.hpp"

#include <tobas_string_tools/core.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_git/core.hpp>
#include <tobas_qt_tools/message.hpp>

namespace gui
{
namespace sa
{
AuthorInformationWidget::AuthorInformationWidget()
{
  name_ = new ParamGetterWidget_LineEdit("Name of the Maintainer", "");
  name_->setValue(QString::fromStdString(git::getGitConfigValue("user.name")));
  addWidget(name_);

  email_ = new ParamGetterWidget_LineEdit("Email of the Maintainer", "");
  email_->setValue(QString::fromStdString(git::getGitConfigValue("user.email")));
  addWidget(email_);

  addStretch();
}

const char* AuthorInformationWidget::name() const
{
  return "Author Info";
}

const char* AuthorInformationWidget::title() const
{
  return "Specify Author Information";
}

const char* AuthorInformationWidget::description() const
{
  return "Enter the name and email address of the person administering the Tobas package "
         "that you're creating with the Setup Assistant. "
         "This step is important for keeping track of package ownership and for any necessary future communications.";
}

void AuthorInformationWidget::onOpened()
{
  return;
}

void AuthorInformationWidget::updateInternalDataStructures()
{
  return;
}

bool AuthorInformationWidget::isValid()
{
  const auto author_name = name_->getValue();
  if (author_name.isEmpty()) {
    qt::qErrorBox(this, "Please specify author name.");
    return false;
  }

  const auto author_email = email_->getValue();
  if (author_email.isEmpty()) {
    qt::qErrorBox(this, "Please specify author email address.");
    return false;
  }
  if (!str::isValidEmail(author_email.toStdString())) {
    qt::qErrorBox(this, "Invalid email address.");
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
