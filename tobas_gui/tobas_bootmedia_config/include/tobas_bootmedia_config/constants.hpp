#pragma once

#include <chrono>

namespace gui
{
namespace bm
{
static constexpr char kPackageName[] = "tobas_bootmedia_config";

static constexpr int kTitlePSize = 18;
static constexpr int kLabelPSize = 12;
static constexpr int kBodyPSize = 9;

static constexpr int kCtrlButtonWidth = 100;
static constexpr int kCtrlButtonHeight = 40;
static constexpr int kFormSpacing = 30;

static constexpr char kUserName[] = "pi";
static constexpr char kBootPath[] = "/mnt/bootfs";
static constexpr char kRootPath[] = "/mnt/rootfs";
static constexpr char kHomePath[] = "/mnt/rootfs/home/pi";

static constexpr int kWpaPskMinLength = 8;
}  // namespace bm
}  // namespace gui
