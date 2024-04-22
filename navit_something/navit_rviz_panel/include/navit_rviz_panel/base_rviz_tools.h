#pragma once
#include <ros/ros.h>

#include "message_navit_map.pb.h"

namespace navit_rviz_tool {
class BaseRvizTool
{
    public:
        virtual bool setItemProperty(const navit::protocol::map_info::MapInfo item_property) = 0;
    
};
} //navit_rviz_tool