#include <tobas_kdl/euler.hpp>

using namespace std;
using namespace KDL;

int main()
{
  const Euler O_Rot_A(M_PI_2, 0, 0);
  const Euler O_Rot_B(M_PI_2, 0, M_PI_2);
  const AngleAxis O_AngleAxis_AB = O_Rot_B - O_Rot_A;
  cout << "Angle-Axis wrt. O: " << O_AngleAxis_AB << endl;
}
