#include "tobas_setup_assistant/tree_information.hpp"

namespace gui
{
namespace setup_assistant
{
TreeInformation::TreeInformation(const kdl::Tree& tree) : super(tree)
{
}

void TreeInformation::updateInternalDataStructures()
{
  super::updateInternalDataStructures();
}

const std::string& TreeInformation::getRootName() const
{
  return tree_.getRootName();
}
}  // namespace setup_assistant
}  // namespace gui
