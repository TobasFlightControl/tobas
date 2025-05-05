#include "tobas_drone_tools/rotor_axis_extractor.hpp"

using namespace std;

namespace tobas
{
RotorAxisExtractor::RotorAxisExtractor(const Drone& drone, const rotor_axis_t& axis) : drone_(drone), axis_(axis)
{
}

bool RotorAxisExtractor::updateInternalDataStructures()
{
  initialize();
  return true;
}

double RotorAxisExtractor::maxThrustSum() const
{
  double res = 0.;
  for (const auto& link_name : link_names_) {
    res += drone_.prop->maxThrust(link_name);
  }
  return res;
}

void RotorAxisExtractor::initialize()
{
  link_names_.clear();

  for (const auto& [link_name, rotor] : drone_.prop->rotors) {
    if (rotor->axis == axis_) {
      link_names_.push_back(link_name);
    }
  }
}
}  // namespace tobas
