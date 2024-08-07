#include "../include/tobas_kdl/treemassholder.hpp"

namespace kdl
{
TreeMassHolder::TreeMassHolder(const Tree& tree) : super(tree), inertia_solver_(tree)
{
}

void TreeMassHolder::updateInternalDataStructures()
{
  inertia_solver_.updateInternalDataStructures();
  inertia_solver_.JntToCart(JntArray::Zero(tree_.getNrOfJoints()));
}
}  // namespace kdl
