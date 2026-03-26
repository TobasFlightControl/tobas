#include "tobas_std_tools/exception.hpp"

using namespace std;

namespace tobas
{
namespace st
{
NotImplementedError::NotImplementedError(const string& msg) : msg_(msg)
{
}

const char* NotImplementedError::what() const noexcept
{
  return msg_.c_str();
}
}  // namespace st
}  // namespace tobas
