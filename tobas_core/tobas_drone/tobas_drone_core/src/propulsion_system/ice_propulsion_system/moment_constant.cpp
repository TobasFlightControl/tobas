#include "tobas_drone_core/propulsion_system/ice_propulsion_system/moment_constant.hpp"

#include <iostream>
#include <limits>

#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

using namespace std;

namespace tobas
{
bool VppMomentConstant::isValid() const
{
  if (a <= 0. || c <= 0.) {
    cerr << "The first and third term of the moment constant must be positive." << endl;
    return false;
  }

  return true;
}

bool VppMomentConstant::load(const YAML::Node& node)
{
  if (!yaml::load(kAKey, node, a)) {
    return false;
  }

  if (!yaml::load(kBKey, node, b)) {
    return false;
  }

  if (!yaml::load(kCKey, node, c)) {
    return false;
  }

  if (!yaml::load(kPhi0Key, node, phi0)) {
    return false;
  }

  return true;
}

YAML::Node VppMomentConstant::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kAKey] = yaml::format(a);
  node[kBKey] = yaml::format(b);
  node[kCKey] = yaml::format(c);
  node[kPhi0Key] = yaml::format(phi0);

  return node;
}

double VppMomentConstant::compute(double phi) const
{
  if (phi <= phi0) {
    cerr << "The moment constant cannot be computed because the pitch angle of " << phi
         << "[rad] is smaller than the negative stall angle of " << phi0 << "[rad]." << endl;
    return numeric_limits<double>::max();
  }

  const auto x = phi - phi0;
  return a * x + b + c / x;
}
}  // namespace tobas
