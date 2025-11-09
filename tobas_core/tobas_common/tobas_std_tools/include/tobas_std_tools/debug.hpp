#pragma once

// 呼ばれた位置のファイル名と行数を表示．
#define PRINT_LOCATION() tbs::_printLocation(__FILE__, __LINE__)

namespace tbs
{
void _printLocation(const char* file, int line);
}  // namespace tbs
