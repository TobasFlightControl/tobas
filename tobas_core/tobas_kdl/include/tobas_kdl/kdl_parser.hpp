#pragma once

#include <string>
#include <urdf_model/model.h>

#include "./tree.hpp"

namespace kdl
{
/** Constructs a tobas_kdl tree from a file, given the file name
 * \param file The filename from where to read the xml
 * \param tree The resulting tobas_kdl Tree
 * returns true on success, false on failure
 */
bool treeFromFile(const std::string& file, Tree& tree);

/** Constructs a tobas_kdl tree from a string containing xml
 * \param xml A string containing the xml description of the robot
 * \param tree The resulting tobas_kdl Tree
 * returns true on success, false on failure
 */
bool treeFromString(const std::string& xml, Tree& tree);

/** Constructs a tobas_kdl tree from a URDF robot model
 * \param robot_model The URDF robot model
 * \param tree The resulting tobas_kdl Tree
 * returns true on success, false on failure
 */
bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, Tree& tree);
}  // namespace kdl
