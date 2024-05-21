#include "dock_visual_perception/Ar.h"

#include <iostream>

namespace dock_visual_perception
{
void arInfo()
{
    std::cerr<< "Start Detection !"<<std::endl;
}

struct ArLoader
{
  ArLoader()
  {
    arInfo();
  }
} arBasicLoader;

}
