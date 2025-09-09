#pragma once

namespace linux
{
bool setRealtimePriorityFIFO(const int& priority);
bool setRealtimePriorityRR(const int& priority);
}  // namespace linux
