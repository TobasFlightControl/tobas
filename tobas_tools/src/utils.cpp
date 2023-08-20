#include <kdl_parser/kdl_parser.hpp>

#include <dh_kdl/treejnttoinertiasolver.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../include/tobas_tools/utils.hpp"

using namespace std;

namespace tobas
{
double getMass()
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromParam(ros::this_node::getNamespace() + "/robot_description", tree))
  {
    rosthrow("Failed to get KDL tree.");
  }

  KDL::TreeJntToInertiaSolver inertia_solver_(tree);
  return inertia_solver_.JntToMass();
}
}  // namespace tobas
