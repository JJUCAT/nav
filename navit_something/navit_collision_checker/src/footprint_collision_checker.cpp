#include <navit_collision_checker/footprint_collision_checker.h>
#include <navit_collision_checker/line_iterator.h>
#include <navit_costmap/cost_values.h>
#include <ros/ros.h>

using namespace navit_costmap;

namespace navit_collision_checker {

    template<typename CostmapT>
    FootprintCollisionChecker<CostmapT>::FootprintCollisionChecker(CostmapT costmap):
        costmap_(costmap)
    {
        
    }

    template<typename CostmapT>
    FootprintCollisionChecker<CostmapT>::FootprintCollisionChecker():
        costmap_(nullptr)
    {
    
    }

    template<typename CostmapT>
    double FootprintCollisionChecker<CostmapT>::footprintCost(const Footprint footprint)
    {
 		// now we really have to lay down the footprint in the costmap_ grid
 		 unsigned int x0, x1, y0, y1;
 		 double footprint_cost = 0.0;

 		 // we need to rasterize each line in the footprint
 		 for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
 		   // get the cell coord of the first point
 		   if (!worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
 		     return static_cast<double>(LETHAL_OBSTACLE);
 		   }

 		   // get the cell coord of the second point
 		   if (!worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
 		     return static_cast<double>(LETHAL_OBSTACLE);
 		   }

 		   footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);
 		 }

 		 // we also need to connect the first point in the footprint to the last point
 		 // get the cell coord of the last point
 		 if (!worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
 		   return static_cast<double>(LETHAL_OBSTACLE);
 		 }

 		 // get the cell coord of the first point
 		 if (!worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
 		   return static_cast<double>(LETHAL_OBSTACLE);
 		 }

 		 footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);

 		 // if all line costs are legal... then we can return that the footprint is legal
 		 return footprint_cost;
    }

    template<typename CostmapT>
    double FootprintCollisionChecker<CostmapT>::lineCost(int x0, int x1, int y0, int y1) const
    {
      double line_cost = 0.0;
  		double point_cost = -1.0;

  		for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
  		  point_cost = pointCost(line.getX(), line.getY());   // Score the current point

  		  if (line_cost < point_cost) {
  		    line_cost = point_cost;
  		  }
  		}

  		return line_cost;    
    }

    template<typename CostmapT>
    void FootprintCollisionChecker<CostmapT>::setCostmap(CostmapT costmap)
    {
        costmap_ = costmap;
    }

    template<typename CostmapT>
    bool FootprintCollisionChecker<CostmapT>::worldToMap(double wx, double wy, 
                                               unsigned int& mx, unsigned int& my)
    {
        if (costmap_ == nullptr)
        {
            ROS_ERROR("costmap is not initalized");
            return false;
        }
        return costmap_->worldToMap(wx, wy, mx, my);
    }

    template<typename CostmapT>
    double FootprintCollisionChecker<CostmapT>::pointCost(int x, int y) const
    {
        return costmap_->getCost(x,y);
    }

    template<typename CostmapT>
    double FootprintCollisionChecker<CostmapT>::footprintCostAtPose(
      double x, double y, double theta, const Footprint footprint)
    {
      double cos_th = cos(theta);
      double sin_th = sin(theta);
      Footprint oriented_footprint;
      for (unsigned int i = 0; i < footprint.size(); ++i) {
        geometry_msgs::Point new_pt;
        new_pt.x = x + (footprint[i].x * cos_th - footprint[i].y * sin_th);
        new_pt.y = y + (footprint[i].x * sin_th + footprint[i].y * cos_th);
        oriented_footprint.push_back(new_pt);
      }
    
      return footprintCost(oriented_footprint);
    }
    
    // declare our valid template parameters
    template class FootprintCollisionChecker<std::shared_ptr<navit_costmap::Costmap2D>>;
    template class FootprintCollisionChecker<navit_costmap::Costmap2D *>;
}
