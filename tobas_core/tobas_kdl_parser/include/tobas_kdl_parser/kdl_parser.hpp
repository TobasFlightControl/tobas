#pragma once

#include <string>

#include <urdf_model/model.h>

#include <tobas_kdl/tree.hpp>

namespace kdl
{
/**
 * @brief Constructs a KDL tree from a file, given the file name.
 *
 * @param path The filename from where to read the xml
 * @param tree The resulting KDL Tree
 *
 * @return true on success, false on failure
 */
bool treeFromPath(const std::string& path, Tree& tree);

/**
 * @brief Constructs a KDL tree from a string containing xml.
 *
 * @param xml A string containing the xml description of the robot
 * @param tree The resulting KDL Tree
 *
 * @return true on success, false on failure
 */
bool treeFromText(const std::string& xml, Tree& tree);

/**
 * @brief Constructs a KDL tree from a URDF robot model.
 *
 * @param robot_model The URDF robot model
 * @param tree The resulting KDL Tree
 *
 * @return true on success, false on failure
 */
bool treeFromUrdf(const urdf::ModelInterface& model, Tree& tree);
}  // namespace kdl
