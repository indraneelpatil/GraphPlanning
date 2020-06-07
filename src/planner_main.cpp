// created by indraneel on 31/05/2020

#include "spdlog/async.h"
#include "spdlog/logger.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include <spdlog/spdlog.h>

#include <ros/ros.h>

#include "occupancy_grid_map.hpp"
#include "breadth_first_search.cpp"
#include "depth_first_search.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>

std::shared_ptr<spdlog::logger> logger;
//Interrupt flag
bool flagLoop = true;
void raiseFlag(int param)
{
    flagLoop = false;
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "graph_planner_node");
  std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>();
  signal(SIGINT, raiseFlag);

  try {
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        "/home/indraneel.p/greyorange_ws/graph_planner_test1.log",
        100 * (1024 * 1024), 3);
    spdlog::init_thread_pool(8192, 1);
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    rotating_sink->set_level(spdlog::level::debug);
    std::vector<spdlog::sink_ptr> sinks{console_sink, rotating_sink};
    auto root_logger = std::make_shared<spdlog::async_logger>(
        "graph_planning", sinks.begin(), sinks.end(), spdlog::thread_pool(),
        spdlog::async_overflow_policy::block);
    root_logger->set_level(spdlog::level::debug);
    spdlog::register_logger(root_logger);
    spdlog::flush_every(std::chrono::seconds(1));
    logger = spdlog::get("graph_planning")->clone("main_thread_logger");
  } catch (const spdlog::spdlog_ex &ex) {
    std::cout << "Log initialization failed: " << ex.what() << std::endl;
  }

  logger->info("initializing graph_planning test!");

  float start_point[2];
  start_point[0] = 0.0f;
  start_point[1] = 0.0f;
  OGMap grid_map(1, 11, 11, start_point,nh_ptr);   // resolution,height.length, start, nh_ptr

  logger->info("Occupancy Grid map created!");

  // Add obstacles 
  bool add_obstacles = true;
  if(add_obstacles)
  {
    grid_map.Map[2][3].value = OGMap::OCCUPIED;
    grid_map.Map[3][4].value = OGMap::OCCUPIED;
    grid_map.Map[4][5].value = OGMap::OCCUPIED;
    grid_map.Map[5][6].value = OGMap::OCCUPIED;
    grid_map.Map[6][8].value = OGMap::OCCUPIED;
    
    logger->info("Obstacles added to the Map!");
    usleep(1000000);
    grid_map.visualise_map();
  }

  // Create Planner
  //BFS planner(grid_map);
   DFS planner(grid_map);

  //ros::AsyncSpinner spinner(1);
  //spinner.start();
  ros::Rate loop_rate(0.2);

  while (flagLoop) {
    
   ros::spinOnce();
  loop_rate.sleep();
  }

  //spinner.stop();

  return 0;
}