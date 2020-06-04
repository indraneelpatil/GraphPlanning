// created by indraneel on 02/06/2020

#ifndef BREADTH_FIRST_SEARCH_HPP
#define BREADTH_FIRST_SEARCH_HPP

#include <iostream>
#include <vector>

#include <spdlog/spdlog.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GridCells.h>
#include <ros/ros.h>

#include "occupancy_grid_map.hpp"

class BFS {

public:
  BFS(float start[], OGMap &OGMap);

  ~BFS();

  void RunPlanner();
  void BuildPathFromQueue();

private:
  std::vector<std::vector<int>> motion_model;
  std::shared_ptr<spdlog::logger> logger_;
  OGMap &MapObj;
  std::condition_variable goal_cv;
  std::mutex goal_mutex_;

  bool planner_alive;
  std::thread planner_thread_;
  std::vector<std::vector<int>> search_queue;
};

#endif