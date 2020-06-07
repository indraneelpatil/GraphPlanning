// created by indraneel on 07/06/2020

#ifndef DEPTH_FIRST_SEARCH_HPP
#define DEPTH_FIRST_SEARCH_HPP

#include <iostream>
#include <vector>
#include <stack>

#include <spdlog/spdlog.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GridCells.h>
#include <ros/ros.h>

#include "occupancy_grid_map.hpp"

class DFS {

public:
  DFS(OGMap &OGMap);

  ~DFS();

  void RunPlanner();
  void BuildPath();
  std::vector<std::vector<int>> feasible_path_coordinates;

private:
  std::vector<std::vector<int>> motion_model;
  std::shared_ptr<spdlog::logger> logger_;
  OGMap &MapObj;
  std::condition_variable goal_cv;
  std::mutex goal_mutex_;

  bool planner_alive;
  std::thread planner_thread_;
  std::vector<std::vector<int>> search_queue;
  std::stack<std::vector<int>> search_stack;   // Stack contains cell indices X,Y 
};

#endif