//
// Created by rakesh on 14/06/18.
//

#include <ground_truth_layer/ground_truth_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ground_truth_layer::GroundTruthLayer, costmap_2d::Layer)

namespace ground_truth_layer
{

GroundTruthLayer::GroundTruthLayer() : is_reinitialized_(false)
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

  local_nh_->param("xmin", origin_x_, -15.0);
  local_nh_->param("ymin", origin_y_, -15.0);
  local_nh_->param("xmax", x_max_, 15.0);
  local_nh_->param("ymax", y_max_, 15.0);

  width_ = (unsigned int)((x_max_ - origin_x_) / resolution_);
  height_ = (unsigned int)((y_max_ - origin_y_) / resolution_);

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
    (width         != width_
    || height      != height_
    || resolution  != resolution_
    || origin_x_meters != origin_x_
    || origin_y_meters != origin_y_)
    && (!layered_costmap_->isSizeLocked())
    )
  {
    layered_costmap_->resizeMap(width_, height_, resolution_, origin_x_, origin_y_);
    resizeMap(width_, height_, resolution_, origin_x_, origin_y_);
    mapper_.initMap(width_, height_, resolution_, origin_x_, origin_y_, costmap_);
    is_reinitialized_ = true;

    ROS_INFO(
      "ground truth costmap resized to: %dx%d, resolution: %f, origin %f, %f from %dx%d, %f, (%f, %f)",
      width_, height_, resolution_, origin_x_, origin_y_,
      width, height, resolution, origin_x_meters, origin_y_meters
    );
  }
}

void GroundTruthLayer::matchSize()
{
  auto master = layered_costmap_->getCostmap();
  auto width = master->getSizeInCellsX();
  auto height = master->getSizeInCellsY();
  auto resolution = master->getResolution();
  auto origin_x = master->getOriginX();
  auto origin_y = master->getOriginY();

  if (
    (width         != width_
     || height      != height_
     || resolution  != resolution_
     || origin_x != origin_x_
     || origin_y != origin_y_)
    && (!layered_costmap_->isSizeLocked())
    )
  {
//    ROS_INFO(
//      "matchSize(): ground truth costmap resized from: %dx%d, resolution: %f, origin %f, %f to %dx%d, %f, (%f, %f)",
//      width_, height_, resolution_, origin_x_, origin_y_,
//      width, height, resolution, origin_x, origin_y
//    );
//
//    width_ = width;
//    height_ = height;
//    resolution_ = resolution;
//
//    layered_costmap_->resizeMap(width_, height_, resolution_, origin_x_, origin_y_);
//    resizeMap(width_, height_, resolution_, origin_x_, origin_y_);
//    mapper_.initMap(width_, height_, resolution_, origin_x_, origin_y_, costmap_);
  }
}


void GroundTruthLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  ROS_DEBUG("update cost requested (%d, %d), (%d, %d)", min_i, min_j, max_i, max_j);
  if (!enabled_)
    return;

  if (!costmap_)
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

  if (is_reinitialized_)
  {
    *min_x = origin_x_;
    *min_y = origin_y_;

    *max_x = x_max_;
    *max_y = y_max_;
    is_reinitialized_ = false;
  }
  else
  {
    auto updated_points_roi = mapper_.getUpdatedPointsROI();
    mapper_.clearUpdatedPoints();

    *min_x = std::min(*min_x, updated_points_roi.x);
    *min_y = std::min(*min_y, updated_points_roi.y);
    *max_x = std::max(*max_x, updated_points_roi.x + updated_points_roi.width);
    *max_y = std::max(*max_y, updated_points_roi.y + updated_points_roi.height);
  }

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

