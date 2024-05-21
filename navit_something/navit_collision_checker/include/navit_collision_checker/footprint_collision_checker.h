#ifndef FOOTPRINT_COLLISION_CHECKER_H
#define FOOTPRINT_COLLISION_CHECKER_H

#include <navit_costmap/costmap_2d.h>

using namespace navit_costmap;

namespace navit_collision_checker {
    typedef std::vector<geometry_msgs::Point> Footprint;

    template<typename CostmapT>
    class FootprintCollisionChecker
    {
    public:
        FootprintCollisionChecker();

        explicit FootprintCollisionChecker(CostmapT costmap);

        double footprintCost(const Footprint footprint);

        bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);

        void setCostmap(CostmapT costmap);

        double lineCost(int x0, int x1, int y0, int y1) const;

        double pointCost(int x, int y) const;
        
        double footprintCostAtPose(double x, double y, double theta, const Footprint footprint);

    protected:
        CostmapT costmap_;
    
    };
}
#endif
