// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstring>
#include <typeinfo>

namespace tobas
{
namespace st
{
/* Class modeled after Python `enum.Enum`. It maps enum values to names. */
struct NamedEnum
{
  const char* name;
  int value;

  explicit NamedEnum()
  {
  }

  constexpr explicit NamedEnum(const char* _name, int _value) : name(_name), value(_value)
  {
  }

  bool operator==(const NamedEnum& other) const
  {
    // Note that comparing pointers such as `char*` with `==` compares addresses, not values.
    return typeid(*this) == typeid(other) && std::strcmp(name, other.name) == 0 && value == other.value;
  }
};
}  // namespace st
}  // namespace tobas

#define DEFINE_NAMED_ENUM(Derived)                                                                                     \
  struct Derived : public st::NamedEnum                                                                                \
  {                                                                                                                    \
    using st::NamedEnum::NamedEnum;                                                                                    \
  };
