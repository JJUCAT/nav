#ifndef BASE_OCCUPANCY_GRID_H_
#define BASE_OCCUPANCY_GRID_H_

#warning navit_core/base_occupancy_grid.h has been deprecated
namespace navit_core {
/**
 * @class Slam
*/
class OccupancyGrid {
  public:
    using Ptr = boost::shared_ptr<OccupancyGrid>;

    virtual void initialize(const std::string& name) = 0;

    virtual ~OccupancyGrid(){}

  protected:
    OccupancyGrid(){}
};

}  // namespace navit_core

#endif  // BASE_OCCUPANCY_GRID_H_
