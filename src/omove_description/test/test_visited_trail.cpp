#include <stdexcept>
#include <iostream>
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl/time.h"
#include "omove_description/trail_cost.hpp"
void check(bool ok, const char * message) {if (!ok) {throw std::runtime_error(message);}}
int main(int argc, char ** argv)
{
  using namespace omove_description;
  check(addTrailCost(255,45)==255,"unknown preserved");
  check(addTrailCost(254,45)==254,"lethal preserved");
  check(addTrailCost(253,45)==253,"inscribed preserved");
  check(addTrailCost(240,45)==252,"soft costs cannot become lethal");
  check(trailPenalty(0,5,.35,5,180,45)==45,"peak");
  check(trailPenalty(0,4,.35,5,180,45)==0,"grace");
  check(trailPenalty(0,180,.35,5,180,45)==0,"expiry");
  check(trailPenalty(.1,90,.35,5,180,45)<45,"spatial temporal decay");
  rclcpp::init(argc,argv);
  {
    auto node=std::make_shared<nav2_util::LifecycleNode>("trail_layer_test", "", rclcpp::NodeOptions());
    auto clock=node->get_clock();
    check(rcl_enable_ros_time_override(clock->get_clock_handle())==RCL_RET_OK,"clock override");
    auto set_time=[&](int sec) {
        check(rcl_set_ros_time_override(clock->get_clock_handle(),int64_t(sec)*1000000000)==RCL_RET_OK,"set clock");
      };
    set_time(10);
    pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader("nav2_costmap_2d","nav2_costmap_2d::Layer");
    nav2_costmap_2d::LayeredCostmap layered("map",false,false);
    layered.resizeMap(100,100,.05,-2.5,-2.5);
    auto layer=loader.createSharedInstance("omove_description::VisitedTrailLayer");
    layer->initialize(&layered,"trail",nullptr,node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    layered.addPlugin(layer);
    auto grid=layered.getCostmap();
    unsigned int i,j; grid->worldToMap(0,0,i,j);
    layered.updateMap(0,0,0);
    check(grid->getCost(i,j)==0,"robot position unpenalized");
    set_time(20); layered.updateMap(.8,0,0);
    check(grid->getCost(i,j)>0 && grid->getCost(i,j)<253,"past trail traversable");
    set_time(200); layered.updateMap(.8,0,0);
    check(grid->getCost(i,j)==0,"expired cost removed from master");
    set_time(210); layered.updateMap(0,0,0);
    set_time(220); layered.updateMap(.8,0,0);
    check(grid->getCost(i,j)>0,"trail restored");
    layer->reset(); layered.updateMap(.8,0,0);
    check(grid->getCost(i,j)==0,"clear removes master cost");
    set_time(230); layered.updateMap(0,0,0);
    set_time(240); layered.updateMap(.8,0,0);
    check(grid->getCost(i,j)>0,"trail before rewind");
    set_time(5); layered.updateMap(.8,0,0);
    check(grid->getCost(i,j)==0,"sim rewind resets trail");
  }
  rclcpp::shutdown();
  std::cout << "Trail plugin integration checks passed\n";
}
