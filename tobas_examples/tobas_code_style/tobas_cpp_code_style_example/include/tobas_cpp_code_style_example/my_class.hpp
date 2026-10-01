// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace my_namespace
{
/** This is a short description that fits on a single line. */
struct MyStruct
{
  /* Public data member only */
  int public_data_member;
};

/**
 * This is a longer description that spans multiple lines because the complete comment,
 * including its delimiters and indentation, exceeds the column limit.
 */
class MyClass
{
public:
  /* Types and type aliases */
  using Ptr = MyClass*;

  enum class ErrorCode
  {
    kNoError,
    kError,
  };

  /** Static constants */
  static constexpr int kStaticConstant = 0;

  /** Factory functions */
  static MyClass FactoryFunction();

  /** Default constructor */
  explicit MyClass();

  /** Move constructor (permit) */
  MyClass(MyClass&& _other) = default;
  MyClass& operator=(MyClass&& _other) = default;

  /** Copy constructor (forbid) */
  MyClass(const MyClass& _other) = delete;
  MyClass& operator=(const MyClass& _other) = delete;

  /** Destructor */
  ~MyClass();

  /* All other functions */
  bool initialize();
  inline int shortMethod() const;

  /**
   * This is a description.
   *
   * @param _primitive_input This is a description.
   * @param _non_primitive_input This is a description.
   * @param _output This is a description.
   * @return This is a description.
   */
  ErrorCode longMethod(int _primitive_input, const std::string& _non_primitive_input, double& _output);

  /* All other data members */
  int public_data_member;

protected:
  int protected_data_member_;  ///< A short member description, when needed.

private:
  /**
   * A longer member description explains constraints or invariants that cannot be inferred from the declaration.
   * Omit the comment when the member is self-explanatory.
   */
  int private_data_member_;
};

inline int MyClass::shortMethod() const
{
  return private_data_member_;
}
}  // namespace my_namespace
}  // namespace tobas
