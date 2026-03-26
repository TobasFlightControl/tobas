#pragma once

#include <chrono>

namespace tobas
{
namespace gui
{
namespace sim
{
static constexpr int kHeaderButtonWidth = 100;
static constexpr int kHeaderButtonHeight = 40;
static constexpr int kCommandButtonHeight = 40;

static constexpr auto kServiceCallTimeout = std::chrono::seconds(1);
}  // namespace sim
}  // namespace gui
}  // namespace tobas
