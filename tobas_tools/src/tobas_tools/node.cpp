#include "../../include/tobas_tools/node.hpp"

namespace tobas
{
BaseNode::BaseNode() : ns_(ros::this_node::getNamespace())
{
}
}  // namespace tobas
