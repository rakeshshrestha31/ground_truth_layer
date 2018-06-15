//
// Created by rakesh on 14/06/18.
//

#include <ground_truth_layer/mapper.h>

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <limits>

// TODO: replace these macros with proper params
#define IS_LOCAL 1
#define LOCAL_X 299
#define LOCAL_Y 299

// how large compared to the original footprint do you want your robot to be
#define ROBOT_SCALE_FACTOR .5

#define MAP_LEAST_COUNT 0.1 // in meters

// how big is the obstacle given by each laser beam (in pixel)
#define LASER_BEAM_WIDTH 1

// we have a gradient between these two colors
#define CURRENT_ROBOT_COLOR 255
#define PREVIOUS_ROBOT_TRAJECTORY_COLOR 100
#define POSITION_GRADIENT_DECAY 0.25

#define OBSTACLE_CHANNEL 0
#define EXPLORED_AREA_CHANNEL 1
#define ROBOT_TRAIL_CHANNEL 2

// #define OBSTACLE_COLOR 127
// #define EXPLORED_AREA_COLOR 30

#define EXPLORED_AREA_COLOR 170
#define RECENTLY_EXPLORED_AREA_COLOR 255
#define EXPLORED_AREA_GRADIENT_DECAY 0.1

#define OBSTACLE_COLOR 250

// in secs
#define RECENTLY_EXPLORED_PIXELS_TIME 0.6

namespace ground_truth_layer
{

void Mapper::reset()
{
  boost::mutex::scoped_lock lock(mutex_);

  robot_footprint_ = cv::Scalar(0, 0, 0);
  map_ = cv::Scalar(0, 0, 0);

  position_gradient_.clear();
  recently_explored_pixels_.clear();
}

void Mapper::initMap(int width, int height, Stg::ModelPosition* robot)
{
  width_ = round(width / (float) MAP_LEAST_COUNT);
  height_ = round(height / (float) MAP_LEAST_COUNT);
  map_ = cv::Mat::zeros(height_, width_, CV_8UC3);
  relative_map_ = cv::Mat::zeros(height_*2, width_*2, CV_8UC3);

  robot_pose_ = robot->GetPose();
  robot_size_ = robot->GetGeom().size;

  robot_grid_size_x_ = round(robot_size_.x / (float) MAP_LEAST_COUNT);
  robot_grid_size_y_ = round(robot_size_.y / (float) MAP_LEAST_COUNT);
  robot_effective_size_ = (robot_grid_size_x_ + robot_grid_size_y_) / 2 * ROBOT_SCALE_FACTOR;

  // since we are going to rotate the robot, with might not fit to (robot_grid_size_x, robot_grid_size_y)
  robot_footprint_ = cv::Mat::zeros(
    robot_grid_size_y_*2*ROBOT_SCALE_FACTOR,
    robot_grid_size_x_*2*ROBOT_SCALE_FACTOR,
    CV_8UC3
  );
  drawRobot();

  // we'll need this over and over again
  one_over_sqrt_2_ = 1. / sqrt(2);
}

int Mapper::drawRobot()
{
  int grid_x;
  int grid_y;

  if (convertToGridCoords(robot_pose_.x, robot_pose_.y, grid_x, grid_y)) {
    return 1;
  }

  // to store the three vertices of the robot triangle
  cv::Point vertices[3];

  // top point
  vertices[0] = cv::Point(
    round(robot_footprint_.cols/2 + cos(robot_pose_.a) * robot_effective_size_),
    round(robot_footprint_.rows/2 + sin(robot_pose_.a) * robot_effective_size_)
  );

  // The bottom points are close to the center so as to maintain a distinct orientation (an isosceles triangle)

  // bottom left point
  vertices[1] = cv::Point(
    round(robot_footprint_.cols/2 + cos(robot_pose_.a + 2./3.*M_PI) * robot_effective_size_*one_over_sqrt_2_),
    round(robot_footprint_.rows/2 + sin(robot_pose_.a + 2./3.*M_PI) * robot_effective_size_*one_over_sqrt_2_)
  );
  // bottom right point
  vertices[2] = cv::Point(
    round(robot_footprint_.cols/2 + cos(robot_pose_.a - 2./3.*M_PI) * robot_effective_size_*one_over_sqrt_2_),
    round(robot_footprint_.rows/2 + sin(robot_pose_.a - 2./3.*M_PI) * robot_effective_size_*one_over_sqrt_2_)
  );

  robot_footprint_ = cv::Scalar(0, 0, 0);
  cv::Scalar robot_color = cv::Scalar(0, 0, 0);
  robot_color[ROBOT_TRAIL_CHANNEL] = CURRENT_ROBOT_COLOR;
  cv::fillConvexPoly(
    robot_footprint_,
    vertices,
    3,
    robot_color
  );

  return 0;
}

cv::Mat Mapper::getCompleteMap()
{
  int robot_grid_x;
  int robot_grid_y;

  if (convertToGridCoords(robot_pose_.x, robot_pose_.y, robot_grid_x, robot_grid_y)) {
    // no robot, return just the map
    return map_;
  }

  cv::Mat map_with_robot = map_.clone();

  for (int rows = 0; rows < robot_footprint_.rows; rows++) {
    for (int cols = 0; cols < robot_footprint_.cols; cols++) {
      if (robot_footprint_.at<cv::Vec3b>(rows, cols)[ROBOT_TRAIL_CHANNEL]) {
        int x = cols - robot_footprint_.cols/2 + robot_grid_x;
        int y = rows - robot_footprint_.rows/2 + robot_grid_y;
        if (
          x >= 0 &&
          y >= 0 &&
          x < map_.cols &&
          y < map_.rows
          ) {
          map_with_robot.at<cv::Vec3b>(y, x)[ROBOT_TRAIL_CHANNEL] = robot_footprint_.at<cv::Vec3b>(rows, cols)[ROBOT_TRAIL_CHANNEL];
          map_with_robot.at<cv::Vec3b>(y, x)[EXPLORED_AREA_CHANNEL] = 0;
          map_with_robot.at<cv::Vec3b>(y, x)[OBSTACLE_CHANNEL] = 0;
        }

      }
    }
  }

  return map_with_robot;
}

cv::Mat Mapper::getMap()
{
  int robot_grid_x;
  int robot_grid_y;

  if (convertToGridCoords(robot_pose_.x, robot_pose_.y, robot_grid_x, robot_grid_y)) {
    // no robot, return just the map
    return map_;
  }

  cv::Mat map_with_robot = getCompleteMap();

  // here we center the map
  float angle = -robot_pose_.a;
  float cos_angle = cos(angle);
  float sin_angle = sin(angle);

  float center_x = (robot_grid_x - width_/2.0);
  float center_y = (robot_grid_y - height_/2.0);

  cv::Mat T1 = cv::Mat::eye(3, 3, CV_32FC1);
  cv::Mat T2 = cv::Mat::eye(3, 3, CV_32FC1);
  cv::Mat T3 = cv::Mat::eye(3, 3, CV_32FC1);
  cv::Mat T4 = cv::Mat::eye(3, 3, CV_32FC1);
  cv::Mat T_composite = cv::Mat::eye(3, 3, CV_32FC1);

  T1.at<float>(0, 2) = width_;
  T1.at<float>(1, 2) = height_;

  T2.at<float>(0, 0) = cos_angle;
  T2.at<float>(0, 1) = -sin_angle;
  T2.at<float>(1, 0) = sin_angle;
  T2.at<float>(1, 1) = cos_angle;

  T3.at<float>(0, 2) = -width_;
  T3.at<float>(1, 2) = -height_;

  T4.at<float>(0, 2) = -center_x;
  T4.at<float>(1, 2) = -center_y;

  T_composite = T1*T2*T3*T4;

  cv::Mat affine_transform = cv::Mat::ones(2, 3, CV_32FC1);

  affine_transform.at<float>(0, 0) = T_composite.at<float>(0, 0);
  affine_transform.at<float>(0, 1) = T_composite.at<float>(0, 1);
  affine_transform.at<float>(0, 2) = T_composite.at<float>(0, 2);
  affine_transform.at<float>(1, 0) = T_composite.at<float>(1, 0);
  affine_transform.at<float>(1, 1) = T_composite.at<float>(1, 1);
  affine_transform.at<float>(1, 2) = T_composite.at<float>(1, 2);

  relative_map_ = cv::Scalar(0, 0, 0);
  cv::Mat tmp = relative_map_(cv::Rect_<int>(width_/2, height_/2, width_, height_));
  map_with_robot.copyTo(tmp);

  cv::Mat tmp_map;
  cv::warpAffine(relative_map_, tmp_map, affine_transform, relative_map_.size());
  relative_map_ = tmp_map.clone();

  if (IS_LOCAL) {
    cv::Mat tmp = relative_map_(cv::Rect_<int>(width_ - LOCAL_X/2, height_ - LOCAL_Y/2, LOCAL_X, LOCAL_Y));
    return tmp.clone();
  }

  return relative_map_;
}

int Mapper::updateMap(Stg::Pose new_robot_pose, const Stg::ModelRanger::Sensor& sensor)
{


#ifdef IS_BASIC_MAP
  // color the whole map just with explored area color (no unexplored pixels)
    cv::Scalar explored_area_color(0, 0, 0);
    explored_area_color[EXPLORED_AREA_CHANNEL] = EXPLORED_AREA_COLOR;
    map_ = explored_area_color;
#endif

  if (updateRobotPose(new_robot_pose)) {
    return 1;
  }

  if (updateLaserScan(sensor)) {
    return 1;
  }

  return 0;
}


int Mapper::updateRobotPose(Stg::Pose new_robot_pose)
{
  boost::mutex::scoped_lock lock(mutex_);

  addPointToPositionGradient();

  robot_pose_ = new_robot_pose;
  return drawRobot();

}

void Mapper::enforcePositionGradientDecay()
{
  for (
    std::map< std::pair<int, int>, double >::iterator it = position_gradient_.begin();
    it != position_gradient_.end();
  ) {
    if (it->second > PREVIOUS_ROBOT_TRAJECTORY_COLOR) {
      int x = it->first.first;
      int y = it->first.second;

      it->second -= POSITION_GRADIENT_DECAY;
      map_.at<cv::Vec3b>(y, x)[ROBOT_TRAIL_CHANNEL] = round(it->second);
      map_.at<cv::Vec3b>(y, x)[EXPLORED_AREA_CHANNEL] = 0;
      map_.at<cv::Vec3b>(y, x)[OBSTACLE_CHANNEL] = 0;
      ++it;
    } else {
      // remove from gradient, they remain fix now
      position_gradient_.erase(it++);
    }
  }
}

int Mapper::addPointToPositionGradient()
{
  enforcePositionGradientDecay();

  int grid_x;
  int grid_y;

  if (convertToGridCoords(robot_pose_.x, robot_pose_.y, grid_x, grid_y)) {
    return 1;
  }

  position_gradient_.insert(
    std::pair< std::pair<int, int>, double >(
      std::make_pair(grid_x, grid_y),
        (double)CURRENT_ROBOT_COLOR - POSITION_GRADIENT_DECAY
    )
  );

  return 0;
}


int Mapper::addPointToRecentlyExploredPixel(int grid_x, int grid_y)
{
  std::pair<int, int> newPixel = std::make_pair(grid_x, grid_y);

  if (
    recently_explored_pixels_.find(newPixel) == recently_explored_pixels_.end() &&
    map_.at<cv::Vec3b>(grid_y, grid_x)[EXPLORED_AREA_CHANNEL] == 0 &&
    map_.at<cv::Vec3b>(grid_y, grid_x)[OBSTACLE_CHANNEL] == 0
    ) {
    recently_explored_pixels_.insert(
      std::pair< std::pair<int, int>, double >(
        newPixel,
          ros::Time::now().toSec()
      )
    );
  }
  return 0;
}

void Mapper::updateRecentlyExploredPixels()
{
  for (
    std::map< std::pair<int, int>, double >::iterator it = recently_explored_pixels_.begin();
    it != recently_explored_pixels_.end();
  ) {

    int x = it->first.first;
    int y = it->first.second;

    if (ros::Time::now().toSec() - it->second > RECENTLY_EXPLORED_PIXELS_TIME) {
      map_.at<cv::Vec3b>(y, x)[OBSTACLE_CHANNEL] = 0;
      map_.at<cv::Vec3b>(y, x)[ROBOT_TRAIL_CHANNEL] = 0;
      map_.at<cv::Vec3b>(y, x)[EXPLORED_AREA_CHANNEL] = EXPLORED_AREA_COLOR;

      recently_explored_pixels_.erase(it++);
    } else {
      map_.at<cv::Vec3b>(y, x)[OBSTACLE_CHANNEL] = 0;
      map_.at<cv::Vec3b>(y, x)[ROBOT_TRAIL_CHANNEL] = 0;
      map_.at<cv::Vec3b>(y, x)[EXPLORED_AREA_CHANNEL] = RECENTLY_EXPLORED_AREA_COLOR;

      it++;
    }
  }
}

int Mapper::updateLaserScan(const Stg::ModelRanger::Sensor& sensor)
{
  // get the data
  const std::vector<Stg::meters_t>& scan = sensor.ranges;

  uint32_t sample_count = scan.size();
  if( sample_count < 1 )
    return 1;

  double laser_orientation = robot_pose_.a - sensor.fov/2.0;
  double angle_increment = sensor.fov/(double)(sensor.sample_count-1);

  for (uint32_t i = 0; i < sample_count; i++) {
    // normalize the angle
    laser_orientation = atan2(sin(laser_orientation), cos(laser_orientation));

    double laser_x, laser_y;
    int laser_grid_x, laser_grid_y;

    laser_x = robot_pose_.x +  scan[i] * cos(laser_orientation);
    laser_y = robot_pose_.y +  scan[i] * sin(laser_orientation);

    if (convertToGridCoords(laser_x, laser_y, laser_grid_x, laser_grid_y)) {
      continue;
    }

    drawScanLine(robot_pose_.x, robot_pose_.y, laser_x, laser_y);

    if ( scan[i] < (sensor.range.max - std::numeric_limits<float>::min()) ) {

      // draw obstacle of size (2.LASER_BEAM_WIDTH X 2.LASER_BEAM_WIDTH) pixels
      for (int row_offset = -LASER_BEAM_WIDTH; row_offset <= LASER_BEAM_WIDTH; row_offset++) {
        int y = laser_grid_y + row_offset;
        if (y >= 0 && y < map_.rows) {
          for (int col_offset = -LASER_BEAM_WIDTH; col_offset <= LASER_BEAM_WIDTH; col_offset++) {
            int x = laser_grid_x + col_offset;
            if (x >= 0 && x < map_.cols) {
              map_.at<cv::Vec3b>(y, x)[OBSTACLE_CHANNEL] = OBSTACLE_COLOR;
              map_.at<cv::Vec3b>(y, x)[EXPLORED_AREA_CHANNEL] = 0;
              map_.at<cv::Vec3b>(y, x)[ROBOT_TRAIL_CHANNEL] = 0;

              // erase if in resently explored pixels
              recently_explored_pixels_.erase( std::make_pair(x, y) );
            }

          }
        }
      }
    }
    laser_orientation += angle_increment;
  }
  updateRecentlyExploredPixels();
  return 0;
}

/********* Utilities ***************/

/*
 @brief convert coords from continuous world coordinate to discrete image coord
*/
int Mapper::convertToGridCoords(double x, double y, int &grid_x, int &grid_y)
{
  grid_x = round( x/(float) MAP_LEAST_COUNT + width_/ 2 );
  grid_y = round( y/(float) MAP_LEAST_COUNT + height_/2 );

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
    if (map_.at<cv::Vec3b>(it.pos())[ROBOT_TRAIL_CHANNEL] < PREVIOUS_ROBOT_TRAJECTORY_COLOR) {
      addPointToRecentlyExploredPixel(it.pos().x, it.pos().y);

      map_.at<cv::Vec3b>(it.pos())[EXPLORED_AREA_CHANNEL] = EXPLORED_AREA_COLOR;

      // if the free space was previously detected as obstacle, decay it slowly
      if (map_.at<cv::Vec3b>(it.pos())[OBSTACLE_CHANNEL]) {
        map_.at<cv::Vec3b>(it.pos())[OBSTACLE_CHANNEL] = 0;
      }
    }
  }

  return 0;

}
} // namespace ground_truth_layer