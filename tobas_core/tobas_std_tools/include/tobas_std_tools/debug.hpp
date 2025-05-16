#pragma once

// 呼ばれた位置のファイル名と行数を表示．
#define PRINT_LOCATION() tobas_std::_printLocation(__FILE__, __LINE__)

namespace tobas_std
{
void _printLocation(const char* file, int line);
}  // namespace tobas_std
