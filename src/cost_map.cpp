// created by indraneel on 7/06/20

#include "cost_map.hpp"

CostMap::CostMap(float resolution, int height, int length, float start[],
                 std::shared_ptr<ros::NodeHandle> nh_ptr, float inf_rad)
    : OGMap(resolution, height, length, start, nh_ptr),
      inflation_radius(inf_rad) {

  logger = spdlog::get("graph_planning")->clone("cost_map_node");

  logger->info("Initialising cost map with default values");

  MapCell cost_cell;
  cost_cell.cost = default_value;

  for (int i = 0; i < GridDim[0]; i++) {
    std::vector<MapCell> row_cells;
    row_cells.clear();
    for (int j = 0; j < GridDim[1]; j++) {
      cost_cell.OGCell = Map[i][j];
      row_cells.push_back(cost_cell);
    }
    cost_map.push_back(row_cells);
  }

}