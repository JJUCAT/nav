#ifndef COSTMAP_2D_TESTING_HELPER_H
#define COSTMAP_2D_TESTING_HELPER_H

#include<navit_costmap/cost_values.h>
#include<navit_costmap/costmap_2d.h>
#include <navit_costmap/static_layer.h>
#include <navit_costmap/obstacle_layer.h>
#include <navit_costmap/inflation_layer.h>

#include <sensor_msgs/point_cloud2_iterator.h>

const double MAX_Z(1.0);

char printableCost(unsigned char cost)
{
  switch (cost)
  {
  case navit_costmap::NO_INFORMATION: return '?';
  case navit_costmap::LETHAL_OBSTACLE: return 'L';
  case navit_costmap::INSCRIBED_INFLATED_OBSTACLE: return 'I';
  case navit_costmap::FREE_SPACE: return '.';
  default: return '0' + (unsigned char) (10 * cost / 255);
  }
}

void printMap(navit_costmap::Costmap2D& costmap)
{
  printf("map:\n");
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      printf("%4d", int(costmap.getCost(j, i)));
    }
    printf("\n\n");
  }
}

unsigned int countValues(navit_costmap::Costmap2D& costmap, unsigned char value, bool equal = true)
{
  unsigned int count = 0;
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      unsigned char c = costmap.getCost(j, i);
      if ((equal && c == value) || (!equal && c != value))
      {
        count+=1;
      }
    }
  }
  return count;
}

void addStaticLayer(navit_costmap::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  navit_costmap::StaticLayer* slayer = new navit_costmap::StaticLayer();
  layers.addPlugin(boost::shared_ptr<navit_costmap::Layer>(slayer));
  slayer->initialize(&layers, "static", &tf);
}

navit_costmap::ObstacleLayer* addObstacleLayer(navit_costmap::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  navit_costmap::ObstacleLayer* olayer = new navit_costmap::ObstacleLayer();
  olayer->initialize(&layers, "obstacles", &tf);
  layers.addPlugin(boost::shared_ptr<navit_costmap::Layer>(olayer));
  return olayer;
}

void addObservation(navit_costmap::ObstacleLayer* olayer, double x, double y, double z = 0.0,
                    double ox = 0.0, double oy = 0.0, double oz = MAX_Z){
  sensor_msgs::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  *iter_x = x;
  *iter_y = y;
  *iter_z = z;

  geometry_msgs::Point p;
  p.x = ox;
  p.y = oy;
  p.z = oz;

  navit_costmap::Observation obs(p, cloud, 100.0, 100.0);  // obstacle range = raytrace range = 100.0
  olayer->addStaticObservation(obs, true, true);
}

navit_costmap::InflationLayer* addInflationLayer(navit_costmap::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  navit_costmap::InflationLayer* ilayer = new navit_costmap::InflationLayer();
  ilayer->initialize(&layers, "inflation", &tf);
  boost::shared_ptr<navit_costmap::Layer> ipointer(ilayer);
  layers.addPlugin(ipointer);
  return ilayer;
}


#endif  // COSTMAP_2D_TESTING_HELPER_H
