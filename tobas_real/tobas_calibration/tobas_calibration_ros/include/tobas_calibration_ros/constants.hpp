#pragma once

#include <cstddef>

namespace tobas_calibration
{
// トピックの送信レート [Hz]．ネットワーク間通信を軽くするために低めに設定している．
static constexpr size_t kPublishRate = 30;
}  // namespace tobas_calibration
