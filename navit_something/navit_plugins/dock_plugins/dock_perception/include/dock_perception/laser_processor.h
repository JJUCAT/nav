#ifndef LASER_PROCESSOR_H
#define LASER_PROCESSOR_H

#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include <math.h>
#include <unistd.h>

#include <algorithm>
#include <list>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "tf/transform_datatypes.h"

namespace laser_processor {
/// A struct representing a single sample from the laser.
class Sample {
public:
  int index;
  float range;
  float intensity;
  float x;
  float y;

  static Sample *Extract(int ind, const sensor_msgs::LaserScan &scan);

private:
  Sample(){};
};

/// The comparator allowing the creation of an ordered "SampleSet"
struct CompareSample {
  CompareSample() {}

  inline bool operator()(const Sample *a, const Sample *b) {
    return (a->index < b->index);
  }
};

/// An ordered set of Samples
class SampleSet : public std::set<Sample *, CompareSample> {
public:
  SampleSet() {}

  ~SampleSet() { clear(); }

  void clear();

  void appendToCloud(sensor_msgs::PointCloud &cloud, int r = 0, int g = 0,
                     int b = 0);

  tf::Point center();
  std_msgs::Header header;
};

/// A mask for filtering out Samples based on range
class ScanMask {
  SampleSet mask_;

  bool filled;
  float angle_min;
  float angle_max;
  uint32_t size;

public:
  ScanMask() : filled(false), angle_min(0), angle_max(0), size(0) {}

  inline void clear() {
    mask_.clear();
    filled = false;
  }

  void addScan(sensor_msgs::LaserScan &scan);

  bool hasSample(Sample *s, float thresh);
};

typedef SampleSet *SampleSetPtr;
typedef SampleSet *SampleSetConstPtr;

class ScanProcessor {
  std::list<SampleSetConstPtr> clusters_;
  sensor_msgs::LaserScan scan_;

public:
  std::list<SampleSetConstPtr> &getClusters() { return clusters_; }

  ScanProcessor(const sensor_msgs::LaserScan &scan, ScanMask &mask_,double x_min,float x_max,float y_min,float y_max,int intensity_threshold=150,
                float mask_threshold = 0.03);

  ~ScanProcessor();

  void removeLessThan(uint32_t num);

  void splitConnected(float thresh);
};
}; // namespace laser_processor

#endif // LASER_PROCESSOR_H
