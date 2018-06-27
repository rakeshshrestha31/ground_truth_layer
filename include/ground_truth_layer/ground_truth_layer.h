//
// Created by rakesh on 14/06/18.
//

#ifndef GROUND_TRUTH_LAYER_GROUND_TRUTH_LAYER_H
#define GROUND_TRUTH_LAYER_GROUND_TRUTH_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <ground_truth_layer/mapper.h>

namespace ground_truth_layer
{
class GroundTruthLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  GroundTruthLayer();
  ~GroundTruthLayer();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                              double* min_y, double* max_x, double* max_y);
  virtual void matchSize();

  bool isDiscretized() { return true; }

  /**
   *
   * @param laser_scan
   * @param odometry
   */
  virtual void updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan,
                 const nav_msgs::OdometryConstPtr &odometry);

  /**
   * @brief fixes map size with respect to master costmap
   */
  void fixMapSize();

protected:
  ros::NodeHandlePtr local_nh_;
  ros::NodeHandlePtr global_nh_;

  boost::shared_ptr< message_filters::Subscriber<sensor_msgs::LaserScan> > laser_sub_;
  boost::shared_ptr< message_filters::Subscriber<nav_msgs::Odometry> > odometry_sub_;
  boost::shared_ptr< message_filters::TimeSynchronizer<sensor_msgs::LaserScan, nav_msgs::Odometry> > topic_sync_;

  std::string odometry_topic_;
  std::string laser_topic_;

  Mapper mapper_;

  double resolution_;
  unsigned int width_;    ///< @brief width of the map
  unsigned int height_;   ///< @brief height of the map

  double origin_x_;
  double origin_y_;
  double x_max_;
  double y_max_;

  unsigned char unknown_cost_value_;    ///< @brief cost assigned to unknown cells
  unsigned char lethal_threshold_;

  bool is_reinitialized_;

  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
} // namespace ground_truth_layer

#endif //GROUND_TRUTH_LAYER_GROUND_TRUTH_LAYER_H
