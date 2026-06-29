// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>
#include <utility>

namespace tobas
{
namespace st
{
namespace detail
{
template <size_t... Is, typename Func>
void staticForImpl(std::index_sequence<Is...>, Func&& f)
{
  (f.template operator()<Is>(), ...);
}
}  // namespace detail

template <size_t N, typename Func>
void staticFor(Func&& f)
{
  detail::staticForImpl(std::make_index_sequence<N>{}, std::forward<Func>(f));
}
}  // namespace st
}  // namespace tobas
