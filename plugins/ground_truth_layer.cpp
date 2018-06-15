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

}

void GroundTruthLayer::updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan,
                                const nav_msgs::OdometryConstPtr &odometry)
{
  // TODO
  ROS_INFO("laser/odom Received!!!");
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

void GroundTruthLayer::matchSize()
{
  // adapted from static layer
  // TODO: preserve previously mapped portions
  auto master_costmap_ptr = layered_costmap_->getCostmap();
  resizeMap(master_costmap_ptr->getSizeInCellsX(), master_costmap_ptr->getSizeInCellsY(), master_costmap_ptr->getResolution(),
            master_costmap_ptr->getOriginX(), master_costmap_ptr->getOriginY());
}

void GroundTruthLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                    double *max_x, double *max_y)
{
  // copied from StaticLayer
  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);
}

} // namespace ground_truth_layer