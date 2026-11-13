// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>
#include <utility>

namespace tobas::st
{
namespace detail
{
template <size_t... Is, typename Func>
void staticForImpl(std::index_sequence<Is...>, Func&& _f)
{
  (_f.template operator()<Is>(), ...);
}
}  // namespace detail

template <size_t N, typename Func>
void staticFor(Func&& _f)
{
  detail::staticForImpl(std::make_index_sequence<N>{}, std::forward<Func>(_f));
}
}  // namespace tobas::st
