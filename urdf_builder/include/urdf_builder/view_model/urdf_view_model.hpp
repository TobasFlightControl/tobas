#pragma once

#include <memory>
#include <urdf/model.h>

#include "./link_view_model.hpp"
#include "./joint_view_model.hpp"

namespace urdf_builder
{
namespace view_model
{
class URDFViewModel
{
public:
  explicit URDFViewModel();

  /* Get URDF model. */
  const urdf::ModelSharedPtr& urdf() const;

  /* Get complete list of Links. */
  const std::map<std::string, urdf::LinkSharedPtr>& links() const;

  /* Get complete list of Joints. */
  const std::map<std::string, urdf::JointSharedPtr>& joints() const;

  /* Get complete list of Materials. */
  const std::map<std::string, urdf::MaterialSharedPtr>& materials() const;

  /* Get the name of the robot model. */
  const std::string& name() const;
  /* Set the name of the robot model. */
  void name(const std::string& name);

  /* Get the root is always a link (the parent of the tree describing the robot). */
  const urdf::LinkSharedPtr& rootLink() const;

  const LinkViewModelPtr& rootLinkViewModel() const;

  QStringList linkNames() const;
  QStringList jointNames() const;

  void newRobot();
  bool loadRobot(const QString& file_path);
  bool saveRobot(const QString& file_path);

  void addLink(const LinkViewModelPtr& link_vm);
  void cloneLink(const LinkViewModelPtr& link_vm);
  void removeLink(const LinkViewModelPtr& link_vm);
  void updateLink(const LinkViewModelPtr& old_link_vm, const LinkViewModelPtr& new_link_vm);

  static void removeTextureTagsWithoutFilename(TiXmlElement* element);

private:
  urdf::ModelSharedPtr urdf_;
  LinkViewModelPtr root_link_;
  size_t clone_count_ = 0;
};
}  // namespace view_model
}  // namespace urdf_builder
