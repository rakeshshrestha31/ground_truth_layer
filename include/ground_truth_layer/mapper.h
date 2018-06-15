//
// Created by rakesh on 14/06/18.
//

#ifndef GROUND_TRUTH_LAYER_MAPPER_H
#define GROUND_TRUTH_LAYER_MAPPER_H

// stdlib includes
#include <map>

// misc includes
#include <boost/thread/mutex.hpp>
#include <boost/atomic.hpp>

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

  Mapper();

  cv::Mat getMapCopy();

  int getWidth() { return width_; }

  int getHeight() { return height_; }

  /**
   *
   * @param width width of map in meters
   * @param height height of map in meters
   * @param resolution meters/pixel in map
   */
  void initMap(int width, int height, float resolution);

  void reset();

  // @return: 1 - success, 0 - failure
  int updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan, const nav_msgs::OdometryConstPtr &robot_odometry);

  int updateLaserScan(const sensor_msgs::LaserScanConstPtr &laser_scan, robot_pose_t robot_pose);

  int drawScanLine(double x1, double y1, double x2, double y2);

  // utilities
  /**
   * @brief convert coords from continuous world coordinate to discrete image coord
   */
  int convertToGridCoords(double x, double y, int &grid_x, int &grid_y);

  static robot_pose_t poseFromGeometryPoseMsg(const geometry_msgs::Pose &pose_msg);


protected:
  boost::mutex mutex_;
  boost::atomic_bool is_initialized_;

  cv::Mat map_;
  cv::Mat relative_map_;

  int width_;
  int height_;
  float resolution_;  ///< @brief meters/pixels

  robot_pose_t robot_pose_;
};
} // namespace ground_truth_layer
#endif //GROUND_TRUTH_LAYER_MAPPER_H
