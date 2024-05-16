#include "../include/tobas_kdl/jacobian.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_kdl
{
void Jacobian::changeRefPoint(const Vector& base_AB)
{
  for (size_t i = 0; i < static_cast<size_t>(data.cols()); ++i)
    setColumn(i, getColumn(i).refPoint(base_AB));
}

bool changeRefPoint(const Jacobian& src1, const Vector& base_AB, Jacobian& dest)
{
  if (src1.columns() != dest.columns())
    return false;
  for (size_t i = 0; i < src1.columns(); ++i)
    dest.setColumn(i, src1.getColumn(i).refPoint(base_AB));
  return true;
}

void Jacobian::changeBase(const Rotation& rot)
{
  for (size_t i = 0; i < static_cast<size_t>(data.cols()); ++i)
    setColumn(i, rot * getColumn(i));
}

bool changeBase(const Jacobian& src1, const Rotation& rot, Jacobian& dest)
{
  if (src1.columns() != dest.columns())
    return false;
  for (size_t i = 0; i < src1.columns(); ++i)
    dest.setColumn(i, rot * src1.getColumn(i));
  return true;
}

void Jacobian::changeRefFrame(const Frame& frame)
{
  for (size_t i = 0; i < static_cast<size_t>(data.cols()); ++i)
    setColumn(i, frame * getColumn(i));
}

bool changeRefFrame(const Jacobian& src1, const Frame& frame, Jacobian& dest)
{
  if (src1.columns() != dest.columns())
    return false;
  for (size_t i = 0; i < src1.columns(); ++i)
    dest.setColumn(i, frame * src1.getColumn(i));
  return true;
}
}  // namespace tobas_kdl
