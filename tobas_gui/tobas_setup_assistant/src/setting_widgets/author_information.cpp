#include <tobas_std_tools/string.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_widgets/author_information.hpp"

namespace gui
{
namespace setup_assistant
{
const char* AuthorInformationWidget::name()
{
  return "Author Info";
}

const char* AuthorInformationWidget::title()
{
  return "Specify Author Information";
}

const char* AuthorInformationWidget::description()
{
  return "Enter the name and email address of the person administering the Tobas package "
         "that you're creating with the Setup Assistant. "
         "This step is important for keeping track of package ownership and for any necessary future communications.";
}

void AuthorInformationWidget::onInit()
{
  name_ = new ParamGetterWidget_LineEdit("Name of the Maintainer", "", QString::fromStdString(linux::userName()));
  rows_->addWidget(name_);

  email_ =
    new ParamGetterWidget_LineEdit("Email of the Maintainer", "", QString::fromStdString(linux::getGitUserEmail()));
  rows_->addWidget(email_);
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
  const auto author_name = name_->get();
  if (author_name.isEmpty())
  {
    qt::qErrorBox(this, "Author name is blank.");
    return false;
  }

  const auto author_email = email_->get();
  if (!tobas_std::isValidEmail(author_email.toStdString()))
  {
    qt::qErrorBox(this, "Invalid email address.");
    return false;
  }

  return true;
}

YAML::Node AuthorInformationWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[name_->name().toStdString()] = name_->get();
  node[email_->name().toStdString()] = email_->get();

  return node;
}

void AuthorInformationWidget::load(const YAML::Node& node)
{
  name_->set(yaml::load<QString>(name_->name().toStdString(), node));
  email_->set(yaml::load<QString>(email_->name().toStdString(), node));
}
}  // namespace setup_assistant
}  // namespace gui
