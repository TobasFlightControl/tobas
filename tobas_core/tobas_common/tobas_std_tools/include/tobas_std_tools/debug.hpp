#pragma once

// 呼ばれた位置のファイル名と行数を表示．
#define PRINT_LOCATION() st::_printLocation(__FILE__, __LINE__)

namespace tobas
{
namespace st
{
void _printLocation(const char* file, int line);
}  // namespace st
}  // namespace tobas
