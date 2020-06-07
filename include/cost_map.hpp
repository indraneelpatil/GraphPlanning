// created by indraneel on 7/06/2020

#ifndef COST_MAP_HPP
#define COST_MAP_HPP

#include <iostream>
#include <spdlog/spdlog.h>
#include <vector>

#include <ros/ros.h>

#include "occupancy_grid_map.hpp"

class CostMap : public OGMap {

public:
  CostMap(float resolution, int height, int length, float start[],
          std::shared_ptr<ros::NodeHandle> nh_ptr, float inflation_radius);

  ~CostMap(){};

  static const unsigned char NO_INFORMATION = 255;
  static const unsigned char LETHAL_OBSTACLE = 254;
  static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
  static const unsigned char FREE_SPACE = 0;

  typedef struct MapCell_ {

    GridCell OGCell;
    unsigned char cost;

  } MapCell;

  std::vector<std::vector<MapCell>> cost_map;

private:
  float inflation_radius;
  unsigned char default_value = FREE_SPACE;
  std::shared_ptr<spdlog::logger> logger;
};
#endif