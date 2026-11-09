#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <vector>
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/empty.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "omove_description/trail_cost.hpp"

namespace omove_description
{
// Recent map-frame positions; sampled by Costmap2DROS after successful pose lookup.
class VisitedTrailLayer : public nav2_costmap_2d::Layer
{
  struct Sample {double x, y, time;};
  std::deque<Sample> samples_;
  std::mutex mutex_;
  double radius_, ttl_, spacing_, grace_, exclusion_, penalty_, x_=0., y_=0.;
  double last_time_=-1.;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_service_;

  void onInitialize() override
  {
    auto node = node_.lock();
    auto param = [&](const std::string & key, double value) {
        declareParameter(key, rclcpp::ParameterValue(value));
        return node->get_parameter(name_ + "." + key).as_double();
      };
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + ".enabled", enabled_);
    radius_=param("trail_radius", 0.35);
    ttl_=param("retention_seconds", 180.0);
    spacing_=param("sample_distance", 0.10);
    grace_=param("grace_seconds", 5.0);
    exclusion_=param("robot_exclusion_radius", 0.5);
    penalty_=param("cost", 45.0);
    if (!(radius_>0 && ttl_>grace_ && grace_>=0 && spacing_>0 &&
      exclusion_>=0 && penalty_>0 && penalty_<=200)) {
      throw std::invalid_argument("Invalid visited trail parameters");
    }
    publisher_=node->create_publisher<nav_msgs::msg::Path>(
      "~/visited_trail", rclcpp::QoS(1).transient_local());
    publisher_->on_activate();
    clear_service_=node->create_service<std_srvs::srv::Empty>(
      "~/clear_visited_trail", [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {reset();});
    current_=true;
  }

public:
  bool isClearable() override {return true;}
  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
    current_=true;
  }

  void updateBounds(double x, double y, double, double * min_x,
    double * min_y, double * max_x, double * max_y) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now=clock_->now().seconds();
    // Sim-time rewind or a localization jump must not paint a fictitious route.
    if (now<last_time_ || (last_time_>=0 && std::hypot(x-x_, y-y_)>1.0)) {
      samples_.clear();
    }
    last_time_=now; x_=x; y_=y;
    while (!samples_.empty() && now-samples_.front().time>=ttl_) {samples_.pop_front();}
    if (enabled_ && (samples_.empty() ||
      std::hypot(x-samples_.back().x, y-samples_.back().y)>=spacing_)) {
      samples_.push_back({x,y,now});
      if (samples_.size()>2000) {samples_.pop_front();}
    }
    // Rebuild the whole small global map so expired/cleared costs disappear.
    // Static, obstacle and inflation layers run first; no persistent master costs.
    auto map=layered_costmap_->getCostmap();
    *min_x=std::min(*min_x,map->getOriginX());
    *min_y=std::min(*min_y,map->getOriginY());
    *max_x=std::max(*max_x,map->getOriginX()+map->getSizeInMetersX());
    *max_y=std::max(*max_y,map->getOriginY()+map->getSizeInMetersY());
    nav_msgs::msg::Path path;
    path.header.frame_id=layered_costmap_->getGlobalFrameID();
    path.header.stamp=clock_->now();
    for (const auto & s : samples_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header=path.header;
      pose.header.stamp=rclcpp::Time(static_cast<int64_t>(s.time*1e9));
      pose.pose.position.x=s.x; pose.pose.position.y=s.y; pose.pose.orientation.w=1.;
      path.poses.push_back(pose);
    }
    publisher_->publish(path);
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & map,
    int min_i, int min_j, int max_i, int max_j) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_) {return;}
    std::vector<unsigned char> trail(map.getSizeInCellsX()*map.getSizeInCellsY(),0);
    const double now=clock_->now().seconds();
    for (const auto & s : samples_) {
      const double age=now-s.time;
      if (age<grace_ || age>=ttl_) {continue;}
      int lo_x,lo_y,hi_x,hi_y;
      map.worldToMapEnforceBounds(s.x-radius_,s.y-radius_,lo_x,lo_y);
      map.worldToMapEnforceBounds(s.x+radius_,s.y+radius_,hi_x,hi_y);
      for (int j=std::max(min_j,lo_y); j<std::min(max_j,hi_y+1); ++j) {
        for (int i=std::max(min_i,lo_x); i<std::min(max_i,hi_x+1); ++i) {
          double wx,wy; map.mapToWorld(i,j,wx,wy);
          if (std::hypot(wx-x_,wy-y_)<exclusion_) {continue;}
          const auto cost=trailPenalty(std::hypot(wx-s.x,wy-s.y),age,
            radius_,grace_,ttl_,penalty_);
          auto & cell=trail[map.getIndex(i,j)];
          cell=std::max(cell,cost); // Overlapping samples never multiply the cost.
        }
      }
    }
    for (int j=min_j;j<max_j;++j) {
      for (int i=min_i;i<max_i;++i) {
        map.setCost(i,j,addTrailCost(map.getCost(i,j),trail[map.getIndex(i,j)]));
      }
    }
    current_=true;
  }
};
}
PLUGINLIB_EXPORT_CLASS(omove_description::VisitedTrailLayer, nav2_costmap_2d::Layer)
