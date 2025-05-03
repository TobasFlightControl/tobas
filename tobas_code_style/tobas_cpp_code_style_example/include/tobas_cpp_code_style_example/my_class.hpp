#pragma once

#include <string>

namespace tobas
{
namespace my_namespace
{
/**
 * @brief This is a short description that fits on a single line.
 */
struct MyStruct
{
  /* Public data member only */
  int public_data_member;
};

/**
 * @brief Although this is a long description spanning multiple lines,
 * you can present it neatly by using \@brief.
 */
class MyClass
{
public:
  /* Types and type aliases */
  using Ptr = MyClass*;

  enum struct ErrorCode
  {
    kNoError,
    kError,
  };

  /* Static constants */
  static constexpr int kStaticConstant = 0;

  /* Factory functions */
  static MyClass FactoryFunction();

  /* Default constructor */
  explicit MyClass();

  /* Move constructor (permit) */
  MyClass(MyClass&& _other) = default;
  MyClass& operator=(MyClass&& _other) = default;

  /* Copy constructor (forbid) */
  MyClass(const MyClass& _other) = delete;
  MyClass& operator=(const MyClass& _other) = delete;

  /* Destructor */
  ~MyClass();

  /* All other functions */
  bool initialize();
  inline int shortMethod() const;

  /**
   * @brief This is a description.
   *
   * @param _primitive_input This is a description.
   * @param _non_primitive_input This is a description.
   * @param _output This is a description.
   *
   * @return This is a description.
   */
  ErrorCode longMethod(int _primitive_input, const std::string& _non_primitive_input, double& _output);

  /* All other data members */
  int public_data_member;

protected:
  int protected_data_member_;  // Variable comments can be written here if needed.

private:
  // Variable comments can be written here if needed.
  int private_data_member_;
};

inline int MyClass::shortMethod() const
{
  return private_data_member_;
}
}  // namespace my_namespace
}  // namespace tobas
