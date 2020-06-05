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
  OGMap(float resolution, int height, int length, float start[],
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
    bool isOnPlannerPath;
    float location[2];
    int parent_index[2];
    int index[2];

  } GridCell;

  std::vector<std::vector<GridCell>> Map;

  void setStartOnGrid(int x, int y);
  void addObstacle(int x, int y);
  void visualise_map();
  void reset_map();
  void print_map_status();

  GridCell *GetCellbyIndex(int x, int y);
  GridCell *GetCellbyPose(float x, float y);
  void GoalCallback(const geometry_msgs::PoseStamped &goal_msg);
  
  bool is_goal_active;
  int GoalCell[2];
  int start_index[2];

private:
  
  float Origin[2];
  int GridDim[2];
  GridCell border_cell;

  int num_explored;
  int num_frontier;

  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<ros::NodeHandle> nh_ptr_;
  
  ros::Publisher free_cell_pub;
  ros::Publisher obstacle_cell_pub;
  ros::Publisher frontier_cell_pub;
  ros::Publisher explored_cell_pub;
  ros::Publisher goal_cell_pub;
  ros::Publisher path_cell_pub;

  ros::Subscriber goal_sub;
};

#endif