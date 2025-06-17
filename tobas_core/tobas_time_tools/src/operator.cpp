#include "tobas_time_tools/operator.hpp"

using namespace std;

ostream& operator<<(ostream& os, const tm& arg)
{
  os << "Year: " << arg.tm_year + 1900 << endl;  // Years since 1900
  os << "Month: " << arg.tm_mon + 1 << endl;     // Months since January [0-11]
  os << "Day: " << arg.tm_mday << endl;
  os << "Hour: " << arg.tm_hour << endl;
  os << "Min: " << arg.tm_min << endl;
  os << "Sec: " << arg.tm_sec << endl;
  return os;
}

double operator-(tm lhs, tm rhs)
{
  const auto time_l = mktime(&lhs);
  const auto time_r = mktime(&rhs);
  return difftime(time_l, time_r);  // sec
}
