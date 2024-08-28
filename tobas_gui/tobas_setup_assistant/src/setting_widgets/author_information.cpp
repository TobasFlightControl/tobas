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

void AuthorInformationWidget::onInit()
{
  name_ = new ParamGetterWidget_LineEdit("Name of the Maintainer", "");
  name_->setValue(QString::fromStdString(linux::userName()));
  addWidget(name_);

  email_ = new ParamGetterWidget_LineEdit("Email of the Maintainer", "");
  email_->setValue(QString::fromStdString(linux::getGitUserEmail()));
  addWidget(email_);
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
  if (author_name.isEmpty())
  {
    qt::qErrorBox(this, "Author name is blank.");
    return false;
  }

  const auto author_email = email_->getValue();
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

  node[name_->name()] = name_->getValue();
  node[email_->name()] = email_->getValue();

  return node;
}

void AuthorInformationWidget::load(const YAML::Node& node)
{
  name_->setValue(yaml::load<QString>(name_->name(), node));
  email_->setValue(yaml::load<QString>(email_->name(), node));
}
}  // namespace setup_assistant
}  // namespace gui
