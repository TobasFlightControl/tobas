#include "../../include/tobas_kdl/conversion/kdl_std.hpp"

using namespace std;

namespace KDL
{
void jntarrayKDLToStd(const JntArray& k, vector<double>& s)
{
  assert(k.rows() == s.size());

  for (size_t i = 0; i < k.rows(); ++i)
  {
    s[i] = k(i);
  }
}

void jntarrayStdToKDL(const vector<double>& s, JntArray& k)
{
  assert(s.size() == k.rows());

  for (size_t i = 0; i < s.size(); ++i)
  {
    k(i) = s[i];
  }
}
}  // namespace KDL
