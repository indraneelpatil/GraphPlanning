// created by indraneel on 31/05/2020

#include "occupancy_grid_map.hpp"

OGMap::OGMap(float resolution, int height_x, int length_y, int start[],
             std::shared_ptr<ros::NodeHandle> nh_ptr)
    : nh_ptr_(nh_ptr) {
  logger_ = spdlog::get("graph_planning")->clone("occupancy_grid_map_node");

  map_publisher =
        nh_ptr_->advertise<nav_msgs::GridCells>("/graph_planning_map", 1);

  // Initialise the map as free
  CellValue free_value = CellValue_::FREE;

  // Initialise cells of map
  GridCell cell;
  cell.value = free_value;
  cell.isExplored = false;
  cell.isFrontier = false;
  cell.isGoal = false;

  // Find origin of map
  Origin[0] = (start[0] - height_x / 2);
  Origin[1] = (start[1] - length_y / 2);

  logger_->info("Origin of map is {},{}", Origin[0], Origin[1]);

  for (int i = 0; i < height_x; i++) {
    std::vector<GridCell> row_cells;
    for (int j = 0; j < length_y; j++) {
      cell.location[0] = Origin[0] + resolution * i;
      cell.location[1] = Origin[1] + resolution * j;
      row_cells.push_back(cell);
    }
    Map.push_back(row_cells);
  }

  GridDim[0] = height_x;
  GridDim[1] = length_y;
  
  usleep(1000000);
  visualise_map();
}

void OGMap::visualise_map() {

    nav_msgs::GridCells grid_cells = nav_msgs::GridCells();
    grid_cells.cell_width = 1;
    grid_cells.cell_height = 1;
    grid_cells.header.stamp = ros::Time::now();
    grid_cells.header.frame_id = "map";

    geometry_msgs::Point grid_cell_point = geometry_msgs::Point();

    for(int i=0;i<GridDim[0];i++)
    {
        for(int j=0;j<GridDim[1];j++)
        {
            grid_cell_point.x = Map[i][j].location[0];
            grid_cell_point.y = Map[i][j].location[1];
            grid_cell_point.z = 0.0f;
            grid_cells.cells.push_back(grid_cell_point);
        }

    }

    map_publisher.publish(grid_cells);
    logger_->debug("Published new map state!");

}