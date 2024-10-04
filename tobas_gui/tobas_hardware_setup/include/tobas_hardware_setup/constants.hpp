#pragma once

#include <chrono>

namespace gui
{
namespace hardware_setup
{
static constexpr char kPackageName[] = "tobas_hardware_setup";
static constexpr char kTitle[] = "Tobas Hardware Setup";

static constexpr int kTitlePSize = 18;
static constexpr int kLabelPSize = 12;
static constexpr int kBodyPSize = 9;

static constexpr auto kSetParamTimeout = std::chrono::seconds(3);

}  // namespace hardware_setup
}  // namespace gui
