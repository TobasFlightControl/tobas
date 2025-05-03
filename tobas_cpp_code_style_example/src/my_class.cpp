// Related header file
#include "tobas_cpp_code_style_example/my_class.hpp"

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
#include "tobas_cpp_code_style_example/util.hpp"

#define MY_MACRO(x) (x)

namespace tobas
{
namespace my_namespace
{
namespace
{
int g_global_variable;

template <typename TypeTemplateParameter>
bool internalMethod(const TypeTemplateParameter& _x)
{
  (void)_x;
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

MyClass::ErrorCode MyClass::longMethod(int _primitive_input, const std::string& _non_primitive_input, double& _output)
{
  (void)_primitive_input;
  (void)_non_primitive_input;
  (void)_output;

  // Put `const` everywhere it can be applied.
  const int local_variable = _primitive_input;

  // Always use braces following `if`, `else`, `do`, `while` and `for` even when the body is a single line.
  if (!internalMethod(local_variable))
  {
    return ErrorCode::kError;
  }

  return ErrorCode::kNoError;
}
}  // namespace my_namespace
}  // namespace tobas
