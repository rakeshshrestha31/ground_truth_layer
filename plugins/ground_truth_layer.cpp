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
  if (!local_nh_ || !global_nh_)
  {
    local_nh_.reset(new ros::NodeHandle("~/" + name_));
    global_nh_.reset(new ros::NodeHandle());
  }

  local_nh_->param("odom_topic",  odometry_topic_, std::string("odom"));
  local_nh_->param("laser_topic", laser_topic_,    std::string("scan"));
  local_nh_->param("resolution",  resolution_,     0.05);

  double xmin, xmax, ymin, ymax;
  local_nh_->param("xmin", xmin, -15.0);
  local_nh_->param("ymin", ymin, -15.0);
  local_nh_->param("xmax", xmax, 15.0);
  local_nh_->param("ymax", ymax, 15.0);

  width_ = (unsigned int)((xmax - xmin) / resolution_);
  height_ = (unsigned int)((ymax - ymin) / resolution_);

  // (re)subscribe only when first time or the topic name changed
  bool is_resubscribed = false;
  if (!laser_sub_ || laser_sub_->getTopic() != ros::names::resolve(laser_topic_))
  {
    laser_sub_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(*global_nh_, laser_topic_, 1));
    is_resubscribed = true;
  }

  if (!odometry_sub_ || odometry_sub_->getTopic() != ros::names::resolve(odometry_topic_))
  {
    odometry_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(*global_nh_, odometry_topic_, 1));
    is_resubscribed = true;
  }

  if (!topic_sync_ || is_resubscribed)
  {
    topic_sync_.reset(
      new message_filters::TimeSynchronizer<sensor_msgs::LaserScan, nav_msgs::Odometry>(
        *laser_sub_,
        *odometry_sub_,
        2
      )
    );
    topic_sync_->registerCallback(boost::bind(&GroundTruthLayer::updateMap, this, _1, _2));
  }

  ROS_INFO("Received a %d X %d map at %f m/pix", width_, height_, resolution_);
  mapper_.initMap(width_, height_, resolution_);
}

void GroundTruthLayer::updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan,
                                const nav_msgs::OdometryConstPtr &odometry)
{
  mapper_.updateMap(laser_scan, odometry);
}

void GroundTruthLayer::reset()
{
  // TODO
  onInitialize();
}

void GroundTruthLayer::activate()
{
  onInitialize();
}

void GroundTruthLayer::deactivate()
{
  if (laser_sub_)
  {
    laser_sub_->unsubscribe();
  }
  if (odometry_sub_)
  {
    odometry_sub_->unsubscribe();
  }
}


} // namespace ground_truth_layer