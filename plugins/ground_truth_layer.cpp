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

  int temp_lethal_threshold, temp_unknown_cost_value;
  local_nh_->param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  local_nh_->param("unknown_cost_value", temp_unknown_cost_value, int(costmap_2d::NO_INFORMATION));

  lethal_threshold_ = (unsigned char)std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = (unsigned char)std::min(temp_unknown_cost_value, 255);

  double xmin, xmax, ymin, ymax;
  local_nh_->param("xmin", xmin, -15.0);
  local_nh_->param("ymin", ymin, -15.0);
  local_nh_->param("xmax", xmax, 15.0);
  local_nh_->param("ymax", ymax, 15.0);

  width_ = (unsigned int)((xmax - xmin) / resolution_);
  height_ = (unsigned int)((ymax - ymin) / resolution_);
  origin_x_ = xmin;
  origin_y_ = ymin;

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

//  auto master_costmap_ptr = layered_costmap_->getCostmap();
//  if ( master_costmap_ptr->getSizeInCellsX()    != width_
//       || master_costmap_ptr->getSizeInCellsY() != height_
//       || master_costmap_ptr->getResolution()   != resolution_
//       || master_costmap_ptr->getOriginX()      != origin_x_
//       || master_costmap_ptr->getOriginY()      != origin_y_
//    )
//  {
//    resizeMap(width_, height_, resolution_, origin_x_, origin_y_);
//    layered_costmap_->resizeMap(width_, height_, resolution_, origin_x_, origin_y_);
//  }

//  layered_costmap_->resizeMap(width_, height_, resolution_, origin_x_, origin_y_);
//  resizeMap(width_, height_, resolution_, origin_x_, origin_y_);
  fixMapSize();
  mapper_.initMap(width_, height_, resolution_, origin_x_, origin_y_, costmap_);

  ROS_INFO("Received a %d X %d map at %f m/pix", width_, height_, resolution_);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(*local_nh_);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
    &GroundTruthLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

}

void GroundTruthLayer::fixMapSize()
{
  auto master_costmap_ptr = layered_costmap_->getCostmap();

  auto width = master_costmap_ptr->getSizeInCellsX();
  auto height = master_costmap_ptr->getSizeInCellsY();
  auto resolution = master_costmap_ptr->getResolution();
  auto origin_x_meters = master_costmap_ptr->getOriginX();
  auto origin_y_meters = master_costmap_ptr->getOriginY();

  if (
    (width          != width_
    || height      != height_
    || resolution  != resolution_
    || origin_x_meters != origin_x_
    || origin_y_meters != origin_y_)
    && (!layered_costmap_->isSizeLocked())
    )
  {
//    width_ = width;
//    height_ = height;
//    resolution_ = resolution;

    layered_costmap_->resizeMap(width_, height_, resolution_, origin_x_, origin_y_);
    resizeMap(width_, height_, resolution_, origin_x_, origin_y_);
    mapper_.initMap(width_, height_, resolution_, origin_x_, origin_y_, costmap_);

    ROS_INFO(
      "ground truth costmap resized to: %dx%d, resolution: %f, origin %f, %f from %dx%d, %f, (%f, %f)",
      width_, height_, resolution_, origin_x_, origin_y_,
      width, height, resolution, origin_x_meters, origin_y_meters
    );
  }
}

void GroundTruthLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  ROS_INFO("update cost requested");
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
//      if (costmap_[index] == costmap_2d::NO_INFORMATION)
//        continue;
      master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

void GroundTruthLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                    double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  unsigned int mx;
  unsigned int my;
  if(worldToMap(mark_x, mark_y, mx, my))
  {
    setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
  }

  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
}

void GroundTruthLayer::matchSize()
{
  auto master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void GroundTruthLayer::updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan,
                                 const nav_msgs::OdometryConstPtr &odometry)
{
  fixMapSize();
  mapper_.updateMap(laser_scan, odometry);
}

void GroundTruthLayer::reset()
{
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

void GroundTruthLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}


} // namespace ground_truth_layer

