// created by indraneel on 31/05/2020

#ifndef OCCUPANCY_GRID_MAP_HPP
#define OCCUPANCY_GRID_MAP_HPP

#include <iostream>
#include <vector>

#include <spdlog/spdlog.h>

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

class OGMap {

public:
  OGMap(float resolution, int height, int length, int start[],
        std::shared_ptr<ros::NodeHandle> nh_ptr);

  ~OGMap(){};

  typedef enum CellValue_ {

    UNKNOWN = -1,
    FREE,
    OCCUPIED
  } CellValue;

  typedef struct GridCell_ {

    CellValue value;
    bool isGoal;
    bool isExplored;
    bool isFrontier;
    float location[2];

  } GridCell;

  std::vector<std::vector<GridCell>> Map;

  void setStartOnGrid(int x, int y);
  void addGoal(int x, int y);
  void addObstacle(int x, int y);
  void visualise_map();

  GridCell *GetCell(float x, float y);
  void GoalCallback(const geometry_msgs::PoseStamped &goal_msg);

private:
  
  float Origin[2];
  int GridDim[2];
  bool is_goal_active;

  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<ros::NodeHandle> nh_ptr_;
  ros::Publisher map_publisher;
  ros::Publisher goal_publisher;
  ros::Subscriber goal_sub;
};

#endif