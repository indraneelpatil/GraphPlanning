// created by indraneel on 31/05/2020

#include "occupancy_grid_map.hpp"

OGMap::OGMap(float resolution, int height_x, int length_y, float start[],
             std::shared_ptr<ros::NodeHandle> nh_ptr)
    : nh_ptr_(nh_ptr), is_goal_active(false) {
  logger_ = spdlog::get("graph_planning")->clone("occupancy_grid_map_node");

  map_publisher =
      nh_ptr_->advertise<nav_msgs::GridCells>("/graph_planning_map", 1);
  goal_publisher =
      nh_ptr_->advertise<nav_msgs::GridCells>("/graph_planning_goal", 1);
  goal_sub = nh_ptr_->subscribe("/move_base_simple/goal", 1,
                                &OGMap::GoalCallback, this);

  // Initialise the map as free
  CellValue free_value = CellValue_::FREE;

  // Initialise cells of map
  GridCell cell;
  cell.value = free_value;
  cell.isExplored = false;
  cell.isFrontier = false;
  cell.isGoal = false;

  // Find origin of map (Centre of map is start point)
  Origin[0] = start[0] - height_x / 2.0f;
  Origin[1] = start[1] - length_y / 2.0f;

  logger_->info("Origin of map is {},{}", Origin[0], Origin[1]);

  for (int i = 0; i < height_x; i++) {
    std::vector<GridCell> row_cells;
    for (int j = 0; j < length_y; j++) {
      cell.location[0] = Origin[0] + resolution * i;
      cell.location[1] = Origin[1] + resolution * j;
      cell.index[0] = i;
      cell.index[1] = j;
      row_cells.push_back(cell);
    }
    Map.push_back(row_cells);
  }

  GridDim[0] = height_x;
  GridDim[1] = length_y;

  // Define a border cell
  border_cell.value = CellValue::OCCUPIED;
  border_cell.isExplored = true;

  usleep(1000000);
  visualise_map();
}

void OGMap::visualise_map() {

  nav_msgs::GridCells grid_cells = nav_msgs::GridCells();
  grid_cells.cell_width = 1.0f;
  grid_cells.cell_height = 1.0f;
  grid_cells.header.stamp = ros::Time::now();
  grid_cells.header.frame_id = "map";

  geometry_msgs::Point grid_cell_point = geometry_msgs::Point();

  for (int i = 0; i < GridDim[0]; i++) {
    for (int j = 0; j < GridDim[1]; j++) {
      grid_cell_point.x = Map[i][j].location[0];
      grid_cell_point.y = Map[i][j].location[1];
      grid_cell_point.z = 0.0f;
      grid_cells.cells.push_back(grid_cell_point);
    }
  }

  map_publisher.publish(grid_cells);
  logger_->debug("Published new map state!");
}

void OGMap::GoalCallback(const geometry_msgs::PoseStamped &goal_msg) {

  // Reject Goal if it is already active
  if (!is_goal_active) {

    GridCell *temp_cell =
        GetCellbyPose(goal_msg.pose.position.x, goal_msg.pose.position.y);
    if (temp_cell->value == CellValue::FREE) {
      temp_cell->isGoal = true;
      is_goal_active = true;

      // Visualise Goal on map
      nav_msgs::GridCells grid_cells = nav_msgs::GridCells();
      grid_cells.cell_width = 1;
      grid_cells.cell_height = 1;
      grid_cells.header.stamp = ros::Time::now();
      grid_cells.header.frame_id = "map";

      geometry_msgs::Point grid_cell_point = geometry_msgs::Point();
      grid_cell_point.x = temp_cell->location[0];
      grid_cell_point.y = temp_cell->location[1];
      grid_cell_point.z = 0.0f;
      grid_cells.cells.push_back(grid_cell_point);
      goal_publisher.publish(grid_cells);
      logger_->info("Got new goal!");

    } else {
      logger_->info("Infeasible Goal Rejected!!");
    }
  }
  else {
      logger_->info("Goal Already active !!");
  }
}

OGMap::GridCell *OGMap::GetCellbyIndex(int x, int y) {

  // Check if this Cell is within dimensions of the map
  if (x > GridDim[0] - 1 || x < 0)
    return &border_cell;

  if (y > GridDim[1] - 1 || y < 0)
    return &border_cell;

  return &Map[(x)][(y)];
}

OGMap::GridCell *OGMap::GetCellbyPose(float x, float y) {

  // Check if this Cell is within dimensions of the map
  if (x > Origin[0] + GridDim[0] || x < Origin[0])
    return &border_cell;

  if (y > Origin[1] + GridDim[1] || y < Origin[1])
    return &border_cell;

  float min_dist = 100.0f;
  GridCell *return_cell;
  // Find cell using Euclidean distance
  for (int i = 0; i < GridDim[0]; i++) {
    for (int j = 0; j < GridDim[1]; j++) {
      float dist = pow(pow(Map[i][j].location[0] - x, 2) +
                           pow(Map[i][j].location[1] - y, 2),
                       0.5);
      if (dist < min_dist) {
        min_dist = dist;
        return_cell = &Map[i][j];
      }
    }
  }

  return return_cell;
}