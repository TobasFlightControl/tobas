// Related header file
#include "tobas_coding_style_example/my_class.hpp"

// C system headers, and any other headers in angle brackets with the .h extension
#include <stdlib.h>
#include <unistd.h>

// C++ standard library headers (without file extension)
#include <algorithm>
#include <cstddef>

// Third-party libraries' header files
#include <eigen3/Eigen/Core>

// Tobas libraries' header files
#include <tobas_math/core.hpp>

// Your project's header files
#include "tobas_coding_style_example/util.hpp"

namespace tobas
{
namespace my_namespace
{
namespace
{
bool internalMethod()
{
  return true;
}
}  // namespace

MyClass MyClass::FactoryFunction()
{
  return MyClass();
}

MyClass::MyClass()
{
  // Constructor must not fail.
}

MyClass::~MyClass()
{
}

bool MyClass::initialize()
{
  return true;
}

MyClass::error_t MyClass::longMethod(int primitive_input, const std::string& non_primitive_input, double& output)
{
  (void)primitive_input;
  (void)non_primitive_input;
  (void)output;

  if (!internalMethod())
    return E_ERROR;

  return E_NO_ERROR;
}
}  // namespace my_namespace
}  // namespace tobas
