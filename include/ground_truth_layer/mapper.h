//
// Created by rakesh on 14/06/18.
//

#ifndef GROUND_TRUTH_LAYER_MAPPER_H
#define GROUND_TRUTH_LAYER_MAPPER_H

// stdlib includes
#include <map>

// misc includes
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>


namespace ground_truth_layer
{
class Mapper
{
public:

  typedef struct
  {
    double x;
    double y;
    double a;
  } robot_pose_t;

  cv::Mat getMapCopy();

  int getWidth() { return width_; }

  int getHeight() { return height_; }

  void initMap(int width, int height, float resolution, const nav_msgs::OdometryConstPtr robot_odometry);

  void reset();

  // @return: 1 - success, 0 - failure
  int updateMap(const nav_msgs::OdometryConstPtr &robot_odometry, const sensor_msgs::LaserScanConstPtr &laser_scan);

  int updateLaserScan(const sensor_msgs::LaserScanConstPtr &laser_scan);

  int drawScanLine(double x1, double y1, double x2, double y2);

  // utilities
  /**
   * @brief convert coords from continuous world coordinate to discrete image coord
   */
  int convertToGridCoords(double x, double y, int &grid_x, int &grid_y);

  static robot_pose_t poseFromGeometryPoseMsg(const geometry_msgs::Pose &pose_msg);


protected:
  boost::mutex mutex_;

  cv::Mat map_;
  cv::Mat relative_map_;

  int width_;
  int height_;
  float resolution_;  ///< @brief meters/pixels

  robot_pose_t robot_pose_;
  static constexpr double one_over_sqrt_2_ = 1. / sqrt(2);

  // to maintain gradient over previous laser scans
  std::map<std::pair<int, int>, double> laser_scan_gradient_;
};
} // namespace ground_truth_layer
#endif //GROUND_TRUTH_LAYER_MAPPER_H
