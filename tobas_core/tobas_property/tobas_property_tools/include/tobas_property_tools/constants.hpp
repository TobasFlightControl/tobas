#pragma once

namespace ptree
{
static constexpr char kGetBoolSrv[] = "get_bool";
static constexpr char kGetIntSrv[] = "get_int";
static constexpr char kGetDoubleSrv[] = "get_double";
static constexpr char kGetStringSrv[] = "get_string";
static constexpr char kSetBoolSrv[] = "set_bool";
static constexpr char kSetIntSrv[] = "set_int";
static constexpr char kSetDoubleSrv[] = "set_double";
static constexpr char kSetStringSrv[] = "set_string";
static constexpr char kSaveFileSrv[] = "save_file";

static constexpr double kWaitForServiceExistence = 3.;  // [s]
}  // namespace ptree
