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
#include <stage.hh>


namespace ground_truth_layer
{
class Mapper
{
public:

  cv::Mat getMapCopy();

  int getWidth() { return width_; }

  int getHeight() { return height_; }

  void initMap(int width, int height, float resolution, Stg::ModelPosition* robot);

  void reset();

  // @output: 0 - success, 1 - failure
  int updateMap(Stg::Pose new_robot_pose, const Stg::ModelRanger::Sensor &sensor);

  int updateRobotPose(Stg::Pose new_robot_pose);

  int updateLaserScan(const Stg::ModelRanger::Sensor &sensor);

  int drawScanLine(double x1, double y1, double x2, double y2);

  // utilities
  /**
   * @brief convert coords from continuous world coordinate to discrete image coord
   */
  int convertToGridCoords(double x, double y, int &grid_x, int &grid_y);


protected:
  boost::mutex mutex_;

  cv::Mat map_;
  cv::Mat relative_map_;

  int width_;
  int height_;
  float resolution_;  ///< @brief meters/pixels

  Stg::Pose robot_pose_;
  Stg::Size robot_size_;

  static constexpr double one_over_sqrt_2_ = 1. / sqrt(2);

  // to maintain gradient over previous laser scans
  std::map<std::pair<int, int>, double> laser_scan_gradient_;
};
} // namespace ground_truth_layer
#endif //GROUND_TRUTH_LAYER_MAPPER_H
