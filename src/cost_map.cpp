// created by indraneel on 7/06/20

#include "cost_map.hpp"

CostMap::CostMap(float resolution, int height, int length, float start[],
                 std::shared_ptr<ros::NodeHandle> nh_ptr, float inf_rad)
    : OGMap(resolution, height, length, start, nh_ptr),
      inflation_radius(inf_rad),cost_scaling_factor(0.3f) {

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
      //cost_cell.OGCell = Map[i][j];
      row_cells.push_back(cost_cell);
    }
    cost_map.push_back(row_cells);
  }

  // Motion model for BFS
  motion_model = {{1, 0}, {0, 1},  {-1, 0},  {0, -1}};

  usleep(1000000);
  view_costmap();
}

void CostMap::view_costmap()
{

  visualization_msgs::MarkerArray map_marker = visualization_msgs::MarkerArray();
  visualization_msgs::Marker map_cell = visualization_msgs::Marker();
  int32_t id = 0;

  map_cell.header.stamp = ros::Time::now();
  map_cell.header.frame_id = "map";
  map_cell.type = visualization_msgs::Marker::CUBE;
  map_cell.lifetime = ros::Duration();

  map_cell.action =  visualization_msgs::Marker::ADD;
  map_cell.pose.orientation.w = 1.0;
  map_cell.mesh_use_embedded_materials = false;

  map_cell.scale.x = 0.9f;
  map_cell.scale.y = 0.9f;
  map_cell.scale.z = 0.1f;

  std_msgs::ColorRGBA color = std_msgs::ColorRGBA();
  for (int i = 0; i < GridDim[0]; i++) {
    for (int j = 0; j < GridDim[1]; j++) {
      map_cell.id = id;
      map_cell.pose.position.x =  Map[i][j].location[0];
      map_cell.pose.position.y = Map[i][j].location[1];
      map_cell.pose.position.z = 0.0;
      uint8_t clr = map_value(cost_map[i][j].cost,50,254,0,254);
      color.r = float(254 - clr)/254.0f;
      color.g = float(254 - clr)/254.0f;
      color.b = float(254 - clr)/254.0f;
      color.a = 1;
      map_cell.color =color;
      map_marker.markers.push_back(map_cell);
      id++;
    }
  }
  
  cost_map_pub.publish(map_marker);
  logger->info("Published new cost map state");

}

uint8_t CostMap::map_value(uint8_t value,uint8_t s1,uint8_t e1,uint8_t s2,uint8_t e2) // 50,254, 0,254
{
  return int(float(s2) + (e2-s2)*(float(value-s1)/float(e1-s1))); //254*(150/204)
}

void CostMap::update_costvalues()
{
  for (int i = 0; i < GridDim[0]; i++) {
    for (int j = 0; j < GridDim[1]; j++) {

      // Look for obstacle cells
      if(Map[i][j].value==OGMap::OCCUPIED)
      {
        cost_map[i][j].cost = LETHAL_OBSTACLE;
        //logger->info("Obstacle cell found");
        //Inflate surrounding cost values
        int cell_indices[2];
        cell_indices[0] = i;
        cell_indices[1] = j;
        inflate_cell(cell_indices);
      }
    }
  }

  view_costmap();
}

/** @brief : Inflate cell using breadth first search */
void CostMap::inflate_cell(int cell_ind[2])
{
  // reset OGMap
  reset_map();

  logger_->info("Inflating obstacle cell found at {},{}",cell_ind[0],cell_ind[1]);
  std::vector<int> current_cell_index;
  current_cell_index.push_back(cell_ind[0]);
  current_cell_index.push_back(cell_ind[1]);
  inflation_queue.push_back(current_cell_index);
  int queue_it = 0;

  //Mark start as a frontier
  Map[inflation_queue[queue_it][0]][inflation_queue[queue_it][1]].isFrontier = true;
  visualise_map();
  while(queue_it<=inflation_queue.size()-1)
  {
    GridCell* current_cell = GetCellbyIndex(inflation_queue[queue_it][0],inflation_queue[queue_it][1]);
    current_cell->isFrontier = false;
    current_cell->isExplored = true;
    //visualise_map();

    for(int i=0;i<motion_model.size();i++)
           {
               // Check if point is not explored and free
                current_cell = GetCellbyIndex(inflation_queue[queue_it][0]+motion_model[i][0],inflation_queue[queue_it][1]+motion_model[i][1]);
                if(!current_cell->isExplored && !current_cell->isFrontier && current_cell->value==OGMap::FREE)
                {
                    //calculate distance from obstacle cell
                     float dist = pow(pow(current_cell->location[0] - Map[cell_ind[0]][cell_ind[1]].location[0], 2) +
                           pow(current_cell->location[1] - Map[cell_ind[0]][cell_ind[1]].location[1], 2),
                           0.5);
                    // Calculate cost
                    float factor = exp(-1.0 * cost_scaling_factor * dist);
                    uint8_t cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
                    // Assign cost
                    uint8_t current_cost = cost_map[current_cell->index[0]][current_cell->index[1]].cost;
                    cost_map[current_cell->index[0]][current_cell->index[1]].cost = std::max(current_cost,cost);
   
                    // Add to frontier
                    current_cell->isFrontier = true;

                    // Add to search queue
                    current_cell_index.clear();
                    current_cell_index.push_back(inflation_queue[queue_it][0]+motion_model[i][0]);
                    current_cell_index.push_back(inflation_queue[queue_it][1]+motion_model[i][1]);
                    inflation_queue.push_back(current_cell_index);

                    //visualise_map();
                    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

           }

    queue_it++;
  }
  //visualise_map();

}

