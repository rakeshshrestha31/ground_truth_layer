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

  cv::Mat getMap();

  cv::Mat getCompleteMap();

  cv::Mat getMapWithoutRobot() { return map_; }

  int getWidth() { return width_; }

  int getHeight() { return height_; }

  void initMap(int width, int height, Stg::ModelPosition *robot);

  void reset();

  // @output: 0 - success, 1 - failure
  int updateMap(Stg::Pose new_robot_pose, const Stg::ModelRanger::Sensor &sensor);

  int updateRobotPose(Stg::Pose new_robot_pose);

  int updateLaserScan(const Stg::ModelRanger::Sensor &sensor);

  int drawRobot();

  int drawScanLine(double x1, double y1, double x2, double y2);

  // for gradient over previous robot positions
  int addPointToPositionGradient();

  void enforcePositionGradientDecay();

  int addPointToRecentlyExploredPixel(int grid_x, int grid_y);

  int removePointFromRecentlyExploredPixel(int grid_x, int grid_y);

  void updateRecentlyExploredPixels();

  // utilities
  int convertToGridCoords(double x, double y, int &grid_x, int &grid_y);


protected:
  boost::mutex mutex_;

  cv::Mat map_;
  cv::Mat relative_map_;

  int width_;
  int height_;

  int robot_grid_size_x_;
  int robot_grid_size_y_;
  int robot_effective_size_;

  Stg::Pose robot_pose_;
  Stg::Size robot_size_;

  double one_over_sqrt_2_;

  // to keep track of what color the robot footprint was before coloring it to CURRENT_ROBOT_COLOR
  cv::Mat robot_footprint_;

  // to maintain gradient over previous robot positions
  std::map<std::pair<int, int>, double> position_gradient_;

  // to maintain gradient over prevous laser scans
  std::map<std::pair<int, int>, double> laser_scan_gradient_;

  // to maintain list of recently explored pixels: key=pixel position, value=timestamp it was added (in secs)
  std::map<std::pair<int, int>, double> recently_explored_pixels_;

};
} // namespace ground_truth_layer
#endif //GROUND_TRUTH_LAYER_MAPPER_H
