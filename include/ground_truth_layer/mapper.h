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
#include <costmap_2d/cost_values.h>

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

  bool isInitialized() { return is_initialized_; }

  /**
   *
   * @param width width of map in meters
   * @param height height of map in meters
   * @param resolution meters/pixel in map
   * @param unknown_cost_value value to initialize the map with (the map is unknown at the beginning)
   */
  void initMap(int width, int height, float resolution,
               double origin_x_meters, double origin_y_meters,
               uint8_t *pointer, unsigned char unknown_cost_value=costmap_2d::NO_INFORMATION);

  /**
   *
   * @param unknown_cost_value value to initialize the map with (the map is unknown at the beginning)
   */
  void reset(unsigned char unknown_cost_value=costmap_2d::NO_INFORMATION);

  // @return: 1 - success, 0 - failure
  int updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan, const nav_msgs::OdometryConstPtr &robot_odometry);

  int updateLaserScan(const sensor_msgs::LaserScanConstPtr &laser_scan, robot_pose_t robot_pose);

  int drawScanLine(int x1, int y1, int x2, int y2);

  // utilities
  /**
   * @brief find the offset in the map to be applied based on costmap offset
   * @param origin_x_meters x-coord of origin of map returned by costmap->getOriginX() in meters
   * @param origin_y_meters y-coord of origin of map returned by costmap->getOriginY() in meters
   * @param offset_x output arg x-coord offset to be applied in map (grid) coords
   * @param offset_y output arg y-coord offset to be applied in map (grid) coords
   */
  static void getMapOffset(double origin_x_meters, double origin_y_meters,
                           double width, double height, double resolution,
                           int &offset_x, int &offset_y);

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

  // origin of the map
  int origin_x_;
  int origin_y_;

  robot_pose_t robot_pose_;
};
} // namespace ground_truth_layer
#endif //GROUND_TRUTH_LAYER_MAPPER_H
