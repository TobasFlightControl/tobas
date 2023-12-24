#include "../include/dh_std_tools/exception.hpp"

using namespace std;

namespace dh_std
{
NotImplementedError::NotImplementedError(const string& msg) : msg_(msg)
{
}

const char* NotImplementedError::what() const noexcept
{
  return msg_.c_str();
}
}  // namespace dh_std
