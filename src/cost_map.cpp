// created by indraneel on 7/06/20

#include "cost_map.hpp"

CostMap::CostMap(float resolution, int height, int length, float start[],
                 std::shared_ptr<ros::NodeHandle> nh_ptr, float inf_rad)
    : OGMap(resolution, height, length, start, nh_ptr),
      inflation_radius(inf_rad) {

  logger = spdlog::get("graph_planning")->clone("cost_map_node");

  cost_map_pub =
      nh_ptr_->advertise<visualization_msgs::MarkerArray>("/graph_planning/cost_map", 1);

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

  usleep(1000000);
  view_costmap();
}

void CostMap::view_costmap()
{

  visualization_msgs::MarkerArray map_marker = visualization_msgs::MarkerArray();
  visualization_msgs::Marker map_cell = visualization_msgs::Marker();

  map_cell.header.stamp = ros::Time::now();
  map_cell.header.frame_id = "map";
  map_cell.type = visualization_msgs::Marker::CUBE;
  map_cell.action =  visualization_msgs::Marker::ADD;
  map_cell.lifetime = ros::Duration();
  map_cell.pose.orientation.w = 1.0;
  map_cell.mesh_use_embedded_materials = false;

  map_cell.scale.x = 0.9f;
  map_cell.scale.y = 0.9f;
  map_cell.scale.z = 0.1f;

  std_msgs::ColorRGBA color = std_msgs::ColorRGBA();
  color.r = 255;
  color.g = 255;
  color.b = 255;
  color.a = 1;
  int32_t id = 0;
  
  for (int i = 0; i < GridDim[0]; i++) {
    for (int j = 0; j < GridDim[1]; j++) {
      map_cell.id = id;
      map_cell.pose.position.x =  cost_map[i][j].OGCell.location[0];
      map_cell.pose.position.y = cost_map[i][j].OGCell.location[1];
      map_cell.pose.position.z = 0.0;
      map_cell.color =color;
      map_marker.markers.push_back(map_cell);
      id++;
    }
  }
  
  cost_map_pub.publish(map_marker);
  logger->info("Published new cost map state");

}