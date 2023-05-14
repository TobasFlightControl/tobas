#include "../../include/tobas_tools/rotor_axis_extractor.hpp"

namespace tobas
{
RotorAxisExtractor::RotorAxisExtractor(const Drone& drone, Axis axis) : drone_(drone), axis_(axis)
{
  updateInternalDataStructures();
}

void RotorAxisExtractor::updateInternalDataStructures()
{
  setRotorIdxs();
}

uint32_t RotorAxisExtractor::count() const
{
  return rotor_idxs_.size();
}

const uint32_t& RotorAxisExtractor::rotorIdx(uint32_t inner_idx) const
{
  return rotor_idxs_[inner_idx];
}

const std::string& RotorAxisExtractor::linkName(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).link_name;
}

const Axis& RotorAxisExtractor::axis(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).axis;
}

const int& RotorAxisExtractor::direction(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).direction;
}

const double& RotorAxisExtractor::motorConstant(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).motor_constant;
}

const double& RotorAxisExtractor::momentConstant(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).moment_constant;
}

const double& RotorAxisExtractor::kv(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).kv;
}

double RotorAxisExtractor::maxRotSpeed(uint32_t inner_idx) const
{
  return drone_.maxRotSpeed(rotorIdx(inner_idx));
}

double RotorAxisExtractor::maxThrust(uint32_t inner_idx) const
{
  return drone_.maxThrust(rotorIdx(inner_idx));
}

double RotorAxisExtractor::thrustToRotSpeed(uint32_t inner_idx, double thrust) const
{
  return drone_.thrustToRotSpeed(rotorIdx(inner_idx), thrust);
}

void RotorAxisExtractor::setRotorIdxs()
{
  rotor_idxs_.clear();

  for (uint32_t i = 0; i < drone_.numRotors(); ++i)
  {
    if (drone_.rotorConfig(i).axis == axis_)
    {
      rotor_idxs_.emplace_back(i);
    }
  }
}
}  // namespace tobas
