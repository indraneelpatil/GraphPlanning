// created by indraneel on 31/05/2020

#include "occupancy_grid_map.hpp"

OGMap::OGMap(float resolution, int height_x, int length_y, float start[],
             std::shared_ptr<ros::NodeHandle> nh_ptr)
    : nh_ptr_(nh_ptr), is_goal_active(false) {
  logger_ = spdlog::get("graph_planning")->clone("occupancy_grid_map_node");

  free_cell_pub =
      nh_ptr_->advertise<nav_msgs::GridCells>("/graph_planning/free_cells", 1);
  obstacle_cell_pub = nh_ptr_->advertise<nav_msgs::GridCells>(
      "/graph_planning/obstacle_cells", 1);
  frontier_cell_pub = nh_ptr_->advertise<nav_msgs::GridCells>(
      "/graph_planning/frontier_cells", 1);
  path_cell_pub =
      nh_ptr_->advertise<nav_msgs::GridCells>("/graph_planning/path_cells", 1);
  explored_cell_pub = nh_ptr_->advertise<nav_msgs::GridCells>(
      "/graph_planning/explored_cells", 1);
  goal_cell_pub =
      nh_ptr_->advertise<nav_msgs::GridCells>("/graph_planning/goal_cell", 1);

  goal_sub = nh_ptr_->subscribe("/move_base_simple/goal", 1,
                                &OGMap::GoalCallback, this);

  // Initialise the map as free
  CellValue free_value = CellValue_::FREE;

  // Initialise cells of map
  GridCell cell;
  cell.value = free_value;
  cell.isExplored = false;
  cell.isFrontier = false;
  cell.isOnPlannerPath = false;
  cell.isGoal = false;

  // Find origin of map (Centre of map is start point)
  if (height_x % 2 == 0) {
    Origin[0] = start[0] - resolution * ((height_x / 2) - 1);
    start_index[0] = (height_x / 2) - 1;

  } else {
    Origin[0] = start[0] - resolution * ((height_x - 1) / 2);
    start_index[0] = (height_x - 1) / 2;
  }

  if (length_y % 2 == 0) {
    Origin[1] = start[1] - resolution * ((length_y / 2) - 1);
    start_index[1] = (length_y / 2) - 1;

  } else {
    Origin[1] = start[1] - resolution * ((length_y - 1) / 2);
    start_index[1] = (length_y - 1) / 2;
  }

  // Origin[0] = start[0] - height_x / 2.0f;
  // Origin[1] = start[1] - length_y / 2.0f;
  logger_->info("Start Index of map is {},{}", start_index[0], start_index[1]);
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
  border_cell.isGoal = false;

  usleep(1000000);
  visualise_map();
}

void OGMap::reset_map() {

  for(int i=0;i<GridDim[0]; i++) {
    for (int j = 0;j<GridDim[1]; j++)
    {
      // Cell values stay the same
      Map[i][j].isExplored = false;
      Map[i][j].isFrontier = false;
      Map[i][j].isOnPlannerPath = false;
      Map[i][j].isGoal = false;
      Map[i][j].parent_index[2] = {0};
    }
  }
  visualise_map();
  logger_->info("Map has been reset!");
}

void OGMap::print_map_status() {

  logger_->info("Explored :{} ===== Frontier :{} ===== Total :{}",num_explored,num_frontier,GridDim[0]*GridDim[1]);
}

void OGMap::visualise_map() {

  nav_msgs::GridCells free_cells = nav_msgs::GridCells();
  nav_msgs::GridCells obstacle_cells = nav_msgs::GridCells();
  nav_msgs::GridCells goal_cell = nav_msgs::GridCells();
  nav_msgs::GridCells frontier_cells = nav_msgs::GridCells();
  nav_msgs::GridCells explored_cells = nav_msgs::GridCells();
  nav_msgs::GridCells path_cells = nav_msgs::GridCells();

  free_cells.cell_width = obstacle_cells.cell_width = goal_cell.cell_width =
      frontier_cells.cell_width = explored_cells.cell_width =
          path_cells.cell_width = 1.0f;
  free_cells.cell_height = obstacle_cells.cell_height = goal_cell.cell_height =
      frontier_cells.cell_height = explored_cells.cell_height =
          path_cells.cell_height = 1.0f;
  free_cells.header.stamp = obstacle_cells.header.stamp =
      goal_cell.header.stamp = frontier_cells.header.stamp =
          explored_cells.header.stamp = path_cells.header.stamp =
              ros::Time::now();
  free_cells.header.frame_id = obstacle_cells.header.frame_id =
      goal_cell.header.frame_id = frontier_cells.header.frame_id =
          explored_cells.header.frame_id = path_cells.header.frame_id = "map";

  geometry_msgs::Point grid_cell_point = geometry_msgs::Point();

  for (int i = 0; i < GridDim[0]; i++) {
    for (int j = 0; j < GridDim[1]; j++) {
      grid_cell_point.x = Map[i][j].location[0];
      grid_cell_point.y = Map[i][j].location[1];
      grid_cell_point.z = 0.0f;

      if (Map[i][j].isOnPlannerPath) {
        path_cells.cells.push_back(grid_cell_point);
      } else if (Map[i][j].isFrontier) {
        frontier_cells.cells.push_back(grid_cell_point);
      } else if (Map[i][j].isGoal) {
        goal_cell.cells.push_back(grid_cell_point);
      } else if (Map[i][j].isExplored) {
        explored_cells.cells.push_back(grid_cell_point);
      } else if (Map[i][j].value == OCCUPIED) {
        obstacle_cells.cells.push_back(grid_cell_point);
      } else {
        free_cells.cells.push_back(grid_cell_point);
      }
    }
  }

  num_explored = explored_cells.cells.size();
  num_frontier = frontier_cells.cells.size();

  if(!free_cells.cells.empty())
    free_cell_pub.publish(free_cells);
  if(!obstacle_cells.cells.empty())
    obstacle_cell_pub.publish(obstacle_cells);
  //if(!frontier_cells.cells.empty())
  frontier_cell_pub.publish(frontier_cells);
  //if(!goal_cell.cells.empty())
  goal_cell_pub.publish(goal_cell);
  //if(!explored_cells.cells.empty())
  explored_cell_pub.publish(explored_cells);
  if(!path_cells.cells.empty())
    path_cell_pub.publish(path_cells);
  logger_->debug("Published new map state!");
}

void OGMap::GoalCallback(const geometry_msgs::PoseStamped &goal_msg) {

  // Reject Goal if it is already active
  if (!is_goal_active) {

    GridCell *temp_cell =
        GetCellbyPose(goal_msg.pose.position.x, goal_msg.pose.position.y);
    if (temp_cell->value == CellValue::FREE) {
      // reset map
      reset_map();
      temp_cell->isGoal = true;
      is_goal_active = true;
      GoalCell[0] = temp_cell->location[0];
      GoalCell[1] = temp_cell->location[1];

      // Visualise Goal on map
      visualise_map();
      logger_->info("Got new goal!");

    } else {
      logger_->info("Infeasible Goal Rejected!!");
    }
  } else {
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