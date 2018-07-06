//
// Created by rakesh on 14/06/18.
//

#include <ground_truth_layer/mapper.h>
#include <ground_truth_layer/config.h>

#include <ros/ros.h>
#include <tf/tf.h>

#include <iostream>
#include <cmath>
#include <limits>

#include <ctime>
#include <chrono>

#include <atomic>
#include <climits>


namespace ground_truth_layer
{

Mapper::Mapper() : is_initialized_(false)
{
}

void Mapper::reset(unsigned char unknown_cost_value)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!map_.empty())
  {
    map_ = cv::Scalar(unknown_cost_value);
  }
  is_initialized_ = false;
}

void Mapper::resetUpdatedArea()
{
  updatedAreaMin_.x = INT_MAX;
  updatedAreaMin_.y = INT_MAX;
  updatedAreaMax_.x = INT_MIN;
  updatedAreaMax_.y = INT_MIN;
}

bool Mapper::isValidUpdateArea()
{
  return !(updatedAreaMin_.x == INT_MAX || updatedAreaMin_.y == INT_MAX ||
    updatedAreaMax_.x == INT_MIN || updatedAreaMax_.y == INT_MIN);
}

cv::Mat Mapper::getMapCopy()
{
  boost::mutex::scoped_lock lock(mutex_);
  return map_.clone();
}

cv::Rect_<double> Mapper::getUpdatedPointsROI()
{
  boost::mutex::scoped_lock lock(mutex_);
  if (isValidUpdateArea())
  {
    cv::Rect grid_coord_roi = cv::Rect(updatedAreaMin_, updatedAreaMax_);

    double x1, y1, x2, y2;
    convertToWorldCoords(grid_coord_roi.x, grid_coord_roi.y, x1, y1);
    convertToWorldCoords(grid_coord_roi.x + grid_coord_roi.width, grid_coord_roi.y + grid_coord_roi.height, x2, y2);

    return cv::Rect_<double>(x1, y1, x2 - x1, y2 - y1);
  }
  else
  {
    return cv::Rect_<double>(1e30, 1e30, -2e30, -2e30);
  }
}

void Mapper::initMap(int width, int height, float resolution,
                     double origin_x_meters, double origin_y_meters,
                     uint8_t *data_pointer, unsigned char unknown_cost_value)
{
  boost::mutex::scoped_lock lock(mutex_);
  resolution_ = resolution;
  
  width_ = width;
  height_ = height;

  getMapOffset(origin_x_meters, origin_y_meters, width_, height_, resolution_, origin_x_, origin_y_);
  ROS_INFO("ground truth map offset: (%d, %d)", origin_x_, origin_y_);

  // TODO: maintain previous map if it exists
  map_ = cv::Mat(height_, width_, CV_8UC1, data_pointer);
  map_ = cv::Scalar(unknown_cost_value);

  is_initialized_ = true;
  
  // tell the parent that the whole costmap has changed (for resetting)
  updatedAreaMin_.x = 0;
  updatedAreaMin_.y = 0;
  updatedAreaMax_.x = width - 1;
  updatedAreaMax_.y = height - 1;
}

int Mapper::updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan, const nav_msgs::OdometryConstPtr &robot_odometry)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!is_initialized_)
  {
    ROS_ERROR("Ground truth map update requested without being initialized");
    return 0;
  }
  robot_pose_ = poseFromGeometryPoseMsg(robot_odometry->pose.pose);

  double mapper_start_us = std::chrono::duration_cast< std::chrono::microseconds >(
    std::chrono::system_clock::now().time_since_epoch()
  ).count();

  if (!updateLaserScan(laser_scan, robot_pose_)) {
    return 0;
  }
  double mapper_end_us = std::chrono::duration_cast< std::chrono::microseconds >(
    std::chrono::system_clock::now().time_since_epoch()
  ).count();

  ROS_DEBUG("mapper took: %f us", mapper_end_us - mapper_start_us);
  // debug
//  cv::Mat map_correct_flip;
//  cv::flip(map_, map_correct_flip, 0);
//  cv::imwrite("/tmp/gt_map.png", map_correct_flip);
  return 1;
}

inline void Mapper::minMaxPoints(const cv::Point& point, cv::Point& min, cv::Point& max)
{
  min.x = std::min(point.x, min.x);
  min.y = std::min(point.y, min.y);

  max.x = std::max(point.x, max.x);
  max.y = std::max(point.y, max.y);
}

int Mapper::updateLaserScan(const sensor_msgs::LaserScanConstPtr &laser_scan, robot_pose_t robot_pose)
{
  // get the data
  const auto &scan = laser_scan->ranges;

  auto sample_count = scan.size();
  if( sample_count < 1 )
    return 1;

  auto laser_beam_orientation_start = robot_pose.a + laser_scan->angle_min;

  int robot_grid_x, robot_grid_y;
  if (!convertToGridCoords(robot_pose.x, robot_pose.y, robot_grid_x, robot_grid_y))
  {
    ROS_ERROR("Robot not within groundtruth map");
    return 0;
  }

  uint32_t i = 0;
  int minx=INT_MAX, miny=INT_MAX, maxx=INT_MIN, maxy=INT_MIN;
  
  #pragma omp parallel for num_threads(NUM_MAPPING_THREADS) reduction(min:minx) reduction(min:miny) reduction(max:maxx) reduction(max:maxy)
  for (i = 0; i < sample_count; i++) {
    // get beam angle and normalize the angle
    auto laser_beam_orientation =laser_beam_orientation_start + i* laser_scan->angle_increment;
    laser_beam_orientation = atan2(sin(laser_beam_orientation), cos(laser_beam_orientation));

    double laser_x, laser_y;
    int laser_grid_x, laser_grid_y;

    laser_x = robot_pose.x +  scan[i] * cos(laser_beam_orientation);
    laser_y = robot_pose.y +  scan[i] * sin(laser_beam_orientation);

    cv::Point local_min = cv::Point(INT_MAX, INT_MAX), local_max = cv::Point(INT_MIN, INT_MIN);

    if (convertToGridCoords(laser_x, laser_y, laser_grid_x, laser_grid_y)) {

      cv::LineIterator it(map_, cv::Point(robot_grid_x, robot_grid_y), cv::Point(laser_grid_x, laser_grid_y));

      // for free space
      for(int j = 0; j < it.count - LASER_BEAM_WIDTH; j++, ++it) {
        auto point = it.pos();
        if (point.x >= 0 && point.x < map_.cols
            && point.y >= 0 && point.y < map_.rows)
        {
          map_.at<uint8_t>(point) = costmap_2d::FREE_SPACE;
          minMaxPoints(point, local_min, local_max);
        }
      }

      // for obstacle space
      if ( scan[i] < (laser_scan->range_max - std::numeric_limits<float>::min()) ) {
          // draw obstacle of size (2.LASER_BEAM_WIDTH X 2.LASER_BEAM_WIDTH) pixels
        for (int row_offset = -LASER_BEAM_WIDTH; row_offset <= LASER_BEAM_WIDTH; row_offset++) {
          int y = laser_grid_y + row_offset;
          if (y >= 0 && y < map_.rows) {
            for (int col_offset = -LASER_BEAM_WIDTH; col_offset <= LASER_BEAM_WIDTH; col_offset++) {
              int x = laser_grid_x + col_offset;
              if (x >= 0 && x < map_.cols) {
                cv::Point point(x, y);
                map_.at<uint8_t>(point) = costmap_2d::LETHAL_OBSTACLE;
                minMaxPoints(point, local_min, local_max);
              }
            }
          }
        }
      }
    }
    minx = std::min(local_min.x, minx);
    miny = std::min(local_min.y, miny);
    maxx = std::max(local_max.x, maxx);
    maxy = std::max(local_max.y, maxy);
  }

  updatedAreaMin_.x = std::min(updatedAreaMin_.x, minx);
  updatedAreaMin_.y = std::min(updatedAreaMin_.y, miny);
  updatedAreaMax_.x = std::max(updatedAreaMax_.x, maxx);
  updatedAreaMax_.y = std::max(updatedAreaMax_.y, maxy);

  return 1;
}

/*************** Utilities ***************/

void Mapper::getMapOffset(double origin_x_meters, double origin_y_meters,
                          double width, double height, double resolution,
                          int &offset_x, int &offset_y)
{
  auto groundtruth_origin_map_x = -width / 2;
  auto groundtruth_origin_map_y = -height / 2;

  auto costmap_origin_map_x = origin_x_meters / resolution;
  auto costmap_origin_map_y = origin_y_meters / resolution;

  // y is negated because y is pointing downwards in opencv coords but upwards in costmap coords
  offset_x = (int)std::round(costmap_origin_map_x - groundtruth_origin_map_x);
  offset_y = -(int)std::round(costmap_origin_map_y - groundtruth_origin_map_y);
};

int Mapper::convertToGridCoords(double x, double y, int &grid_x, int &grid_y)
{
  grid_x = (int)round( x/resolution_ + width_/ 2 - origin_x_ );
  grid_y = (int)round( y/resolution_ + height_/2 - origin_y_ );

  if (
    grid_x < 0 ||
    grid_y < 0 ||
    grid_x > width_ ||
    grid_y > height_
    ) {
    return 0;
  } else {
    return 1;
  }
}

int Mapper::convertToWorldCoords(int grid_x, int grid_y, double &x, double &y)
{
  x = (grid_x + origin_x_ - width_/2.0) * resolution_;
  y = (grid_y + origin_y_ - height_/2.0) * resolution_;

  return 1;
}

Mapper::robot_pose_t Mapper::poseFromGeometryPoseMsg(const geometry_msgs::Pose &pose_msg)
{
  return {
    pose_msg.position.x,
    pose_msg.position.y,
    tf::getYaw(pose_msg.orientation)
  };
}

} // namespace ground_truth_layer
