#pragma once

#include <cstring>
#include <typeinfo>

namespace tobas_std
{
/* Pythonのenum.Enumを模したクラス．列挙型の番号と名前を対応付けることができる． */
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
    // ポインタ (ここではchar*) 同士を==などで比較すると，値ではなくアドレスの比較になってしまうことに注意．
    return typeid(*this) == typeid(other) && std::strcmp(name, other.name) == 0 && value == other.value;
  }
};
}  // namespace tobas_std

#define DEFINE_NAMED_ENUM(Derived)                                                                                     \
  struct Derived : public tobas_std::NamedEnum                                                                         \
  {                                                                                                                    \
    using tobas_std::NamedEnum::NamedEnum;                                                                             \
  };
