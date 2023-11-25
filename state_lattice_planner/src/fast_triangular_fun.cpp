/**
 * @copyright Copyright (c) {2022} LZY
 * @author LZY (linziyan@yijiahe.com)
 * @date 2023-11-17
 * @brief 
 */

#include <local_planner/fast_triangular_fun.h>
#include <cmath>

namespace fast_triangular {

float fast_sin(float x) {
  x = x / M_PI * 180.0;

  int sig = 0;

  if(x > 0.0){
      while(x >= 360.0) {
          x = x - 360.0;
      }
  }else{
      while(x < 0.0) {
          x = x + 360.0;
      }
  }

  if(x >= 180.0){
      sig = 1;
      x = x - 180.0;
  }

  x = (x > 90.0) ? (180.0 - x) : x;

  int a = x * 0.1;
  float b = x - 10 * a;
  
  float y = sin_table[a] * cos_table[(int)b] + b * hollyst * sin_table[9 - a];

  return (sig > 0) ? -y : y;
}

float fast_cos(float x)
{
  float a_sin = x + M_PI_2;
  // a_sin = a_sin > M_2PI ? a_sin - M_2PI : a_sin;
  return fast_sin( a_sin );
}

}
