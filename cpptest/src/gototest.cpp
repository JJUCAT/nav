#include <cpptest/gototest.h>
#include <bits/stdc++.h>

namespace cpptest {


bool Goto::Work(const int arg0, const int arg1, const int arg2)
{
  if (arg0 != 0) {
    std::cout << "config arg0 (" << arg0 << ") error !" << std::endl;
    sum = 0.0;
    return false;
  }

  if (arg1 != 1) {
    std::cout << "config arg1 (" << arg0 << ") error !" << std::endl;
    sum = 0.0;
    return false;
  }

  if (arg2 != 2) {
    std::cout << "config arg2 (" << arg0 << ") error !" << std::endl;
    sum = 0.0;
    return false;
  }

  sum = arg0 + arg1 + arg2;
  return true;
}


} // namespace cpptest
