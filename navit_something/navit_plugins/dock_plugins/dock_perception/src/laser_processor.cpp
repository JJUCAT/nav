#include <dock_perception/laser_processor.h>
#include <stdexcept>

#include <list>
#include <set>

namespace laser_processor {
Sample *Sample::Extract(int ind, const sensor_msgs::LaserScan &scan) {
  Sample *s = new Sample();

  s->index = ind;
  s->range = scan.ranges[ind];
  s->x = cos(scan.angle_min + ind * scan.angle_increment) * s->range;
  s->y = sin(scan.angle_min + ind * scan.angle_increment) * s->range;
  s->intensity=scan.intensities[ind];
  if (s->range > scan.range_min && s->range < scan.range_max)
    return s;
  else {
    delete s;
    return NULL;
  }
}

void SampleSet::clear() {
  for (SampleSet::iterator i = begin(); i != end(); i++) {
    delete (*i);
  }
  set<Sample *, CompareSample>::clear();
}

void SampleSet::appendToCloud(sensor_msgs::PointCloud &cloud, int r, int g,
                              int b) {
  float color_val = 0;

  int rgb = (r << 16) | (g << 8) | b;
  color_val = *reinterpret_cast<float *>(&rgb);

  for (iterator sample_iter = begin(); sample_iter != end(); sample_iter++) {
    geometry_msgs::Point32 point;
    point.x = (*sample_iter)->x;
    point.y = (*sample_iter)->y;
    point.z = 0;

    cloud.points.push_back(point);

    if (cloud.channels[0].name == "rgb")
      cloud.channels[0].values.push_back(color_val);
  }
}

tf::Point SampleSet::center() {
  float x_mean = 0.0;
  float y_mean = 0.0;
  for (iterator i = begin(); i != end(); i++)

  {
    x_mean += ((*i)->x) / size();
    y_mean += ((*i)->y) / size();
  }

  return tf::Point(x_mean, y_mean, 0.0);
}

void ScanMask::addScan(sensor_msgs::LaserScan &scan) {
  if (!filled) {
    angle_min = scan.angle_min;
    angle_max = scan.angle_max;
    size = scan.ranges.size();
    filled = true;
  } else if (angle_min != scan.angle_min || angle_max != scan.angle_max ||
             size != scan.ranges.size()) {
    throw std::runtime_error("laser_scan::ScanMask::addScan: inconsistantly "
                             "sized scans added to mask");
  }

  for (uint32_t i = 0; i < scan.ranges.size(); i++) {
    Sample *s = Sample::Extract(i, scan);

    if (s != NULL) {
      SampleSet::iterator m = mask_.find(s);

      if (m != mask_.end()) {
        if ((*m)->range > s->range) {
          delete (*m);
          mask_.erase(m);
          mask_.insert(s);
        } else {
          delete s;
        }
      } else {
        mask_.insert(s);
      }
    }
  }
}

bool ScanMask::hasSample(Sample *s, float thresh) {
  if (s != NULL) {
    SampleSet::iterator m = mask_.find(s);
    if (m != mask_.end())
      if (((*m)->range - thresh) < s->range)
        return true;
  }
  return false;
}

ScanProcessor::ScanProcessor(const sensor_msgs::LaserScan &scan,
                             ScanMask &mask_,double x_min,float x_max,float y_min,float y_max, int intensity_threshold,float mask_threshold) {
  scan_ = scan;

  SampleSet *cluster = new SampleSet;
  cluster->header = scan.header;

  for (uint32_t i = 0; i < scan.ranges.size(); i++) {
    Sample *s = Sample::Extract(i, scan);

    if (s != NULL) {
      if (!mask_.hasSample(s, mask_threshold)&&(s->x>=x_min&&s->x<=x_max&&s->y>=y_min&&s->y<=y_max)&&s->intensity>=intensity_threshold) {
        cluster->insert(s);
      } else {
        delete s;
      }
    }
  }

  clusters_.push_back(cluster);
}

ScanProcessor::~ScanProcessor() {
  for (std::list<SampleSet *>::iterator c = clusters_.begin();
       c != clusters_.end(); c++)
    delete (*c);
}

void ScanProcessor::removeLessThan(uint32_t num) {
  std::list<SampleSet *>::iterator c_iter = clusters_.begin();
  while (c_iter != clusters_.end()) {
    if ((*c_iter)->size() < num) {
      delete (*c_iter);
      clusters_.erase(c_iter++);
    } else {
      ++c_iter;
    }
  }
}

void ScanProcessor::splitConnected(float thresh) {
  std::list<SampleSet *> tmp_clusters;

  std::list<SampleSet *>::iterator c_iter = clusters_.begin();

  // For each cluster
  while (c_iter != clusters_.end()) {
    // Go through the entire list
    while ((*c_iter)->size() > 0) {
      // Take the first element
      SampleSet::iterator s_first = (*c_iter)->begin();

      // Start a new queue
      std::list<Sample *> sample_queue;
      sample_queue.push_back(*s_first);

      (*c_iter)->erase(s_first);

      // Grow until we get to the end of the queue
      std::list<Sample *>::iterator s_q = sample_queue.begin();
      while (s_q != sample_queue.end()) {
        int expand = static_cast<int>(asin(thresh / (*s_q)->range) /
                                      scan_.angle_increment);

        SampleSet::iterator s_rest = (*c_iter)->begin();

        while ((s_rest != (*c_iter)->end() &&
                (*s_rest)->index < (*s_q)->index + expand)) {
          if ((*s_rest)->range - (*s_q)->range > thresh) {
            break;
          } else if (sqrt(pow((*s_q)->x - (*s_rest)->x, 2.0f) +
                          pow((*s_q)->y - (*s_rest)->y, 2.0f)) < thresh) {
            sample_queue.push_back(*s_rest);
            (*c_iter)->erase(s_rest++);
            break;
          } else {
            ++s_rest;
          }
        }
        s_q++;
      }

      // Move all the samples into the new cluster
      SampleSet *c = new SampleSet;
      c->header = (*c_iter)->header;
      for (s_q = sample_queue.begin(); s_q != sample_queue.end(); s_q++)
        c->insert(*s_q);

      // Store the temporary clusters
      tmp_clusters.push_back(c);
    }

    // Now that c_iter is empty, we can delete
    delete (*c_iter);

    // And remove from the map
    clusters_.erase(c_iter++);
  }

  clusters_.insert(clusters_.begin(), tmp_clusters.begin(), tmp_clusters.end());
}

}; // namespace laser_processor
