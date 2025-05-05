#include "tobas_drone_core/fixed_wing/vehicle_params.hpp"

#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_yaml_tools/convert/range.hpp>

using namespace std;

namespace tobas
{
bool VehicleParameters::isValid() const
{
  if (wing_surface <= 0) {
    cerr << "Wing surface must be positive." << endl;
    return false;
  }

  if (wing_span <= 0) {
    cerr << "Wing span must be positive." << endl;
    return false;
  }

  if (mac <= 0) {
    cerr << "Mean aerodynamic chord must be positive." << endl;
    return false;
  }

  if (!alpha_limit.isValid()) {
    cerr << "Invalid stall angles." << endl;
    return false;
  }

  return true;
}

bool VehicleParameters::load(const YAML::Node& node)
{
  if (!yaml::load(kWingSurfaceKey, node, wing_surface)) {
    return false;
  }

  if (!yaml::load(kWingSpanKey, node, wing_span)) {
    return false;
  }

  if (!yaml::load(kMACKey, node, mac)) {
    return false;
  }

  if (!yaml::load(kAeroCenterKey, node, ac.data)) {
    return false;
  }

  if (!yaml::load(kAlphaLimitLKey, node, alpha_limit)) {
    return false;
  }

  return true;
}

YAML::Node VehicleParameters::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kWingSurfaceKey] = wing_surface;
  node[kWingSpanKey] = wing_span;
  node[kMACKey] = mac;
  node[kAeroCenterKey] = ac.data;
  node[kAlphaLimitLKey] = alpha_limit;

  return node;
}
}  // namespace tobas
