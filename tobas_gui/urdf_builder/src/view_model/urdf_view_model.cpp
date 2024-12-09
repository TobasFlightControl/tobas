#include <queue>
#include <urdf_parser/urdf_parser.h>

#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/console.hpp>

#include "../../include/urdf_builder/view_model/urdf_view_model.hpp"

using namespace std;

namespace urdf_builder
{
namespace view_model
{
URDFViewModel::URDFViewModel() : urdf_(new urdf::Model())
{
}

const urdf::ModelSharedPtr& URDFViewModel::urdf() const
{
  return urdf_;
};

const map<string, urdf::LinkSharedPtr>& URDFViewModel::links() const
{
  return urdf_->links_;
}

const map<string, urdf::JointSharedPtr>& URDFViewModel::joints() const
{
  return urdf_->joints_;
}

const map<string, urdf::MaterialSharedPtr>& URDFViewModel::materials() const
{
  return urdf_->materials_;
}

const string& URDFViewModel::name() const
{
  return urdf_->name_;
}

void URDFViewModel::name(const string& name)
{
  urdf_->name_ = name;
}

const urdf::LinkSharedPtr& URDFViewModel::rootLink() const
{
  return urdf_->root_link_;
}

const LinkViewModelPtr& URDFViewModel::rootLinkViewModel() const
{
  return root_link_;
}

QStringList URDFViewModel::linkNames() const
{
  QStringList result;
  transform(
    urdf_->links_.begin(), urdf_->links_.end(), back_inserter(result),
    [](const pair<string, urdf::LinkSharedPtr>& pair) { return QString::fromStdString(pair.first); });
  return result;
}

QStringList URDFViewModel::jointNames() const
{
  QStringList result;
  transform(
    urdf_->joints_.begin(), urdf_->joints_.end(), back_inserter(result),
    [](const pair<string, urdf::JointSharedPtr>& pair) { return QString::fromStdString(pair.first); });
  return result;
}

void URDFViewModel::newRobot()
{
  urdf_.reset(new urdf::Model());
  root_link_ = nullptr;
  clone_count_ = 0;
}

bool URDFViewModel::loadRobot(const QString& file_path)
{
  newRobot();

  if (!urdf_->initFile(file_path.toStdString()))
  {
    PRINT_ERROR("Failed to parse URDF.");
    urdf_ = nullptr;
    return false;
  }

  root_link_.reset(new LinkViewModel(urdf_->root_link_));
  return true;
}

bool URDFViewModel::saveRobot(const QString& file_path)
{
  // URDFを修正してXMLに変換
  urdf_->root_link_->inertial = nullptr;      // ルートリンクのイナーシャを削除
  const auto doc = urdf::exportURDF(*urdf_);  // TiXmlは生ポインタで扱うのが基本
  removeTextureTagsWithoutFilename(doc->RootElement());

  // XMLを保存
  if (doc->SaveFile(file_path.toStdString().c_str()) != tinyxml2::XML_SUCCESS)
  {
    PRINT_ERROR("Failed to save URDF: " << doc->ErrorStr());
    return false;
  }

  return true;
}

void URDFViewModel::addLink(const LinkViewModelPtr& link_vm)
{
  const auto& joint_vm = link_vm->joint();

  const auto link_name = link_vm->name().toStdString();
  const auto joint_name = joint_vm->name().toStdString();
  const auto parent_link_name = joint_vm->parentLinkName().toStdString();

  if (link_name.empty())
    throw runtime_error("Link name cannot be empty.");

  if (joint_name.empty())
    throw runtime_error("Joint name cannot be empty.");

  if (urdf_->links_.find(link_name) != urdf_->links_.end())
    throw runtime_error("Link \"" + link_name + "\" already exists.");

  if (urdf_->joints_.find(joint_name) != urdf_->joints_.end())
    throw runtime_error("Joint \"" + joint_name + "\" already exists.");

  if (parent_link_name.empty())
  {
    if (urdf_->root_link_ != nullptr)
      throw runtime_error(
        "The root link already exists, but the parent link of \"" + link_name + "\" is not specified.");

    urdf_->root_link_ = link_vm->model();
    urdf_->joints_.erase(urdf_->root_link_->parent_joint->name);
    urdf_->root_link_->parent_joint = nullptr;
    urdf_->root_link_->inertial = nullptr;  // ルートリンクはイナーシャを持てない
    root_link_.reset(new LinkViewModel(urdf_->root_link_));
  }
  else
  {
    if (urdf_->links_.find(parent_link_name) == urdf_->links_.end())
      throw runtime_error("Parent link \"" + parent_link_name + "\" does not exist.");

    const auto& parent_link = urdf_->links_.at(parent_link_name);
    auto& child_links = parent_link->child_links;
    auto& child_joints = parent_link->child_joints;
    if (!tobas_std::contains(child_links, link_vm->model()))
      child_links.push_back(link_vm->model());
    if (!tobas_std::contains(child_joints, joint_vm->model()))
      child_joints.push_back(joint_vm->model());
  }

  urdf_->links_[link_name] = link_vm->model();
  urdf_->joints_[joint_name] = joint_vm->model();

  for (const auto& visual : link_vm->visuals())
  {
    const auto& material_vm = visual->material();
    urdf_->materials_[material_vm->name().toStdString()] = material_vm->model();
  }

  for (const auto& child_link_vm : link_vm->children())
    addLink(child_link_vm);
}

void URDFViewModel::cloneLink(const LinkViewModelPtr& link_vm)
{
  const auto clone = link_vm->clone();
  const auto suffix = "_" + QString::number(++clone_count_);
  addNameSuffixRec(clone, suffix);
  addLink(clone);
}

void URDFViewModel::removeLink(const LinkViewModelPtr& link_vm)
{
  const auto& link = link_vm->model();
  assert(link != urdf_->root_link_);  // ルートリンクを消すとバグる

  const auto& parent_link = urdf_->links_[link->parent_joint->parent_link_name];
  auto& child_links = parent_link->child_links;
  auto& child_joints = parent_link->child_joints;
  child_links.erase(remove(child_links.begin(), child_links.end(), link), child_links.end());
  child_joints.erase(remove(child_joints.begin(), child_joints.end(), link->parent_joint), child_joints.end());

  queue<urdf::LinkSharedPtr> que;
  que.push(link);

  while (!que.empty())
  {
    auto top = que.front();
    que.pop();

    urdf_->links_.erase(top->name);
    if (top->parent_joint)
      urdf_->joints_.erase(top->parent_joint->name);

    for (const auto& visual : top->visual_array)
      urdf_->materials_.erase(visual->material_name);

    for (const auto& child : top->child_links)
      que.push(child);
  }
}

void URDFViewModel::updateLink(const LinkViewModelPtr& old_link_vm, const LinkViewModelPtr& new_link_vm)
{
  // remove old
  const auto& old_joint = old_link_vm->joint();

  urdf_->links_.erase(old_link_vm->name().toStdString());
  urdf_->joints_.erase(old_joint->name().toStdString());
  for (const auto& visual : old_link_vm->visuals())
    urdf_->materials_.erase(visual->material()->name().toStdString());

  const auto& old_parent_link_it = urdf_->links_.find(old_joint->parentLinkName().toStdString());
  if (old_parent_link_it != urdf_->links_.end())
  {
    const auto& old_parent_link = old_parent_link_it->second;

    auto& child_links = old_parent_link->child_links;
    const auto& it1 = remove_if(
      child_links.begin(), child_links.end(),
      [&](const urdf::LinkSharedPtr& link) { return link->name == old_link_vm->name().toStdString(); });
    child_links.erase(it1, child_links.end());

    auto& child_joints = old_parent_link->child_joints;
    const auto& it2 = remove_if(
      child_joints.begin(), child_joints.end(),
      [&](const urdf::JointSharedPtr& joint) { return joint->name == old_joint->name().toStdString(); });
    child_joints.erase(it2, child_joints.end());
  }

  // add new
  const auto& new_joint = new_link_vm->joint();
  const auto& new_parent_link_it = urdf_->links_.find(new_joint->parentLinkName().toStdString());
  if (new_parent_link_it != urdf_->links_.end())
  {
    const auto& new_parent_link = new_parent_link_it->second;
    new_parent_link->child_links.push_back(new_link_vm->model());
    new_parent_link->child_joints.push_back(new_link_vm->joint()->model());
  }

  urdf_->joints_[new_joint->name().toStdString()] = new_joint->model();
  urdf_->links_[new_link_vm->name().toStdString()] = new_link_vm->model();
  for (const auto& visual : new_link_vm->visuals())
    urdf_->materials_[visual->material()->name().toStdString()] = visual->material()->model();

  if (new_link_vm->model() == urdf_->root_link_)
  {
    urdf_->root_link_ = new_link_vm->model();
    root_link_.reset(new LinkViewModel(urdf_->root_link_));
  }
}

void URDFViewModel::addNameSuffixRec(const LinkViewModelPtr& link_vm, const QString& suffix)
{
  const auto& joint_vm = link_vm->joint();

  const auto new_link_name = link_vm->name() + suffix;
  const auto new_joint_name = joint_vm->name() + suffix;

  link_vm->name(new_link_name);
  joint_vm->name(new_joint_name);
  joint_vm->childLinkName(new_link_name);

  for (const auto& child_link_vm : link_vm->children())
  {
    child_link_vm->joint()->parentLinkName(new_link_name);
    addNameSuffixRec(child_link_vm, suffix);
  }
}

void URDFViewModel::removeTextureTagsWithoutFilename(tinyxml2::XMLElement* element)
{
  if (element == nullptr)
    return;

  for (auto child = element->FirstChildElement(); child != nullptr; child = child->NextSiblingElement())
  {
    // FIXME: 繰り返し中にツリー構造を変えるのはまずいかも．対象要素をリストしておいて後で消すべき．
    if (string(child->Name()) == "texture" && !child->Attribute("filename"))
      element->DeleteChild(child);

    // 再帰的に子要素もチェック
    removeTextureTagsWithoutFilename(child);
  }
}
}  // namespace view_model
}  // namespace urdf_builder
