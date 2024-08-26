#pragma once

#include <tobas_kdl/treesolveri.hpp>

namespace gui
{
namespace setup_assistant
{
class TreeInformation : public kdl::TreeSolverI
{
  using super = kdl::TreeSolverI;

public:
  explicit TreeInformation(const kdl::Tree& tree);

  void updateInternalDataStructures() override;

  const std::string& getRootName() const;
};
}  // namespace setup_assistant
}  // namespace gui
