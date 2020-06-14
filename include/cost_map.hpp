// created by indraneel on 7/06/2020

#ifndef COST_MAP_HPP
#define COST_MAP_HPP

#include <iostream>
#include <spdlog/spdlog.h>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include "occupancy_grid_map.hpp"

#define COST_FACTOR 0.8

class CostMap : public OGMap {

public:
  CostMap(float resolution, int height, int length, float start[],
          std::shared_ptr<ros::NodeHandle> nh_ptr, float inflation_radius);

  ~CostMap(){};

  static const unsigned char NO_INFORMATION = 255;
  static const unsigned char LETHAL_OBSTACLE = 254;
  static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
  static const unsigned char FREE_SPACE = 0;
  static const unsigned char COST_NEUTRAL = 50;

  /** navfn cost convention
   * cell value = cost*cost_factor + cost_neutral 
   */

  typedef struct MapCell_ {

    //GridCell OGCell;
    unsigned char cost;

  } MapCell;

  std::vector<std::vector<MapCell>> cost_map;

  void view_costmap();
  uint8_t map_value(uint8_t value,uint8_t s1,uint8_t e1,uint8_t s2,uint8_t e2);
  void update_costvalues();
  void inflate_cell(int cell_ind[2]);

private:
  float inflation_radius;
  unsigned char default_value = COST_NEUTRAL;
  std::shared_ptr<spdlog::logger> logger;
  ros::Publisher cost_map_pub;
  std::vector<std::vector<int>> inflation_queue;
  std::vector<std::vector<int>> motion_model; //for BFS
  float cost_scaling_factor;
};
#endif