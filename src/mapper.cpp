//
// Created by rakesh on 14/06/18.
//

#include <ground_truth_layer/mapper.h>

#include <ros/ros.h>
#include <costmap_2d/cost_values.h>
#include <tf/tf.h>

#include <iostream>
#include <cmath>
#include <limits>

// how big is the obstacle given by each laser beam (in pixel)
#define LASER_BEAM_WIDTH 1

namespace ground_truth_layer
{

void Mapper::reset()
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!map_.empty())
  {
    map_ = cv::Scalar(0, 0, 0);
  }
}

cv::Mat Mapper::getMapCopy()
{
  boost::mutex::scoped_lock lock(mutex_);
  return map_.clone();
}

void Mapper::initMap(int width, int height, float resolution, const nav_msgs::OdometryConstPtr robot_odometry)
{
  resolution_ = resolution;
  
  width_ = (int)round(width / resolution_);
  height_ = (int)round(height / resolution_);
  map_ = cv::Mat::zeros(height_, width_, CV_8UC1);

  robot_pose_ = poseFromGeometryPoseMsg(robot_odometry->pose.pose);
}

int Mapper::updateMap(const nav_msgs::OdometryConstPtr &robot_odometry, const sensor_msgs::LaserScanConstPtr &laser_scan)
{
  boost::mutex::scoped_lock lock(mutex_);
  robot_pose_ = poseFromGeometryPoseMsg(robot_odometry->pose.pose);

  if (updateLaserScan(laser_scan)) {
    return 0;
  }

  return 1;
}

int Mapper::updateLaserScan(const sensor_msgs::LaserScanConstPtr &laser_scan)
{
  // get the data
  const auto &scan = laser_scan->ranges;

  auto sample_count = scan.size();
  if( sample_count < 1 )
    return 1;

  auto laser_beam_orientation = robot_pose_.a - laser_scan->angle_min;

  for (uint32_t i = 0; i < sample_count; i++) {
    // normalize the angle
    laser_beam_orientation = atan2(sin(laser_beam_orientation), cos(laser_beam_orientation));

    double laser_x, laser_y;
    int laser_grid_x, laser_grid_y;

    laser_x = robot_pose_.x +  scan[i] * cos(laser_beam_orientation);
    laser_y = robot_pose_.y +  scan[i] * sin(laser_beam_orientation);

    if (convertToGridCoords(laser_x, laser_y, laser_grid_x, laser_grid_y)) {
      continue;
    }

    // TODO: parallelize ray casting
    drawScanLine(robot_pose_.x, robot_pose_.y, laser_x, laser_y);

    if ( scan[i] < (laser_scan->range_max - std::numeric_limits<float>::min()) ) {
      // draw obstacle of size (2.LASER_BEAM_WIDTH X 2.LASER_BEAM_WIDTH) pixels
      for (int row_offset = -LASER_BEAM_WIDTH; row_offset <= LASER_BEAM_WIDTH; row_offset++) {
        int y = laser_grid_y + row_offset;
        if (y >= 0 && y < map_.rows) {
          for (int col_offset = -LASER_BEAM_WIDTH; col_offset <= LASER_BEAM_WIDTH; col_offset++) {
            int x = laser_grid_x + col_offset;
            if (x >= 0 && x < map_.cols) {
              map_.at<cv::Vec3b>(y, x) = costmap_2d::LETHAL_OBSTACLE;
            }
          }
        }
      }
    }

    laser_beam_orientation += laser_scan->angle_increment;
  }
  return 1;
}

/*************** Utilities ***************/

int Mapper::convertToGridCoords(double x, double y, int &grid_x, int &grid_y)
{
  grid_x = (int)round( x/resolution_ + width_/ 2 );
  grid_y = (int)round( y/resolution_ + height_/2 );

  if (
    grid_x < 0 ||
    grid_y < 0 ||
    grid_x  > width_ ||
    grid_y > height_
    ) {
    return 1;
  } else {
    return 0;
  }
}

int Mapper::drawScanLine(double x1, double y1, double x2, double y2)
{

  int grid_x1, grid_y1, grid_x2, grid_y2;

  if ( convertToGridCoords(x1, y1, grid_x1, grid_y1) ) {
    return 1;
  }

  if ( convertToGridCoords(x2, y2, grid_x2, grid_y2) ) {
    return 1;
  }

  cv::LineIterator it(map_, cv::Point(grid_x1, grid_y1), cv::Point(grid_x2, grid_y2));

  for(int i = 0; i < it.count; i++, ++it) {
    map_.at<cv::Vec3b>(it.pos()) = costmap_2d::FREE_SPACE;
  }

  return 0;
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