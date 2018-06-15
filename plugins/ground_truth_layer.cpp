//
// Created by rakesh on 14/06/18.
//

#include <ground_truth_layer/ground_truth_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ground_truth_layer::GroundTruthLayer, costmap_2d::Layer)

namespace ground_truth_layer
{

GroundTruthLayer::GroundTruthLayer()
{
  // TODO
}

GroundTruthLayer::~GroundTruthLayer()
{
  // TODO
}

void GroundTruthLayer::onInitialize()
{
  local_nh_.reset(new ros::NodeHandle("~/" + name_));
  global_nh_.reset(new ros::NodeHandle());

  local_nh_->param("odom_topic",  odometry_topic_, std::string("odom"));
  local_nh_->param("laser_topic", laser_topic_,    std::string("scan"));

  laser_sub_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(*global_nh_, laser_topic_, 1));
  odometry_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(*global_nh_, odometry_topic_, 1));
  topic_sync_.reset(
    new message_filters::TimeSynchronizer<sensor_msgs::LaserScan, nav_msgs::Odometry>(
      *laser_sub_,
      *odometry_sub_,
      2
    )
  );
  topic_sync_->registerCallback(boost::bind(&GroundTruthLayer::callback, this, _1, _2));
}

void GroundTruthLayer::callback(const sensor_msgs::LaserScanConstPtr &laser_scan,
                                const nav_msgs::OdometryConstPtr &odometry)
{
  ROS_INFO("Received!!!");
}

} // namespace ground_truth_layer