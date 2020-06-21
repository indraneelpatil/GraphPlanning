// created by indraneel on 17/06/2020

#include "dijkstras_planner.hpp"

Dijkstras::Dijkstras(CostMap &CostMap) : MapObj(CostMap), planner_alive(true) {

  logger_ = spdlog::get("graph_planning")->clone("dijkstras_node");

  motion_model = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

  // Start Planner
  planner_thread_ = std::thread(&Dijkstras::RunPlanner, this);
}

Dijkstras::~Dijkstras() {

     planner_alive = false;
    planner_thread_.join();
}

void Dijkstras::RunPlanner()
{

    logger_->info("Waiting for Goal!");
    while (planner_alive) {
        // Wait to receive start point
        std::unique_lock<std::mutex> goal_lock(goal_mutex_);
        goal_cv.wait_for(goal_lock, std::chrono::milliseconds(2000),
                        [this] { return MapObj.is_goal_active; });

        if (MapObj.is_goal_active) {
            logger_->info("Goal Received!");
            
            // Add start to the queue
            OGMap::GridCell* current_cell = MapObj.GetCellbyIndex(MapObj.start_index[0],MapObj.start_index[1]);
            std::pair<int,CostMap::GridCell> cc_pair = std::make_pair(0,*current_cell);
            queue.push(cc_pair);

            //Mark start as a frontier
            current_cell->isFrontier = true;
            // End of queue marker
            current_cell->parent_index[0] = -1;
            current_cell->parent_index[1] = -1;
            MapObj.view_costmap();
            MapObj.visualise_map();
            while(planner_alive&&!queue.empty()){

                // Get cell with minimum cost from the priority queue
                cc_pair = queue.top();
                queue.pop();
                current_cell = MapObj.GetCellbyIndex(cc_pair.second.index[0],cc_pair.second.index[1]);
                // Remove from frontier add to explored
                current_cell->isFrontier = false;
                current_cell->isExplored = true;

                MapObj.view_costmap();
                MapObj.visualise_map();

                for(int i=0;i<motion_model.size();i++)
                {
                    // Check if point is not explored and free
                        current_cell = MapObj.GetCellbyIndex(cc_pair.second.index[0]+motion_model[i][0],cc_pair.second.index[1]+motion_model[i][1]);
                        if(!current_cell->isExplored && current_cell->value==OGMap::FREE)
                        {
                            // Check if it is already in Frontier
                            if(current_cell->isFrontier)
                            {
                                // cost = cost_so_far + cost_cell
                                int new_cost = cc_pair.first+ MapObj.cost_map[current_cell->index[0]][current_cell->index[1]].cost;
                                // Check current cost in priority queue
                                if(CompareCost(new_cost,*current_cell))
                                {
                                    //Update parent cell of current cell
                                    current_cell->parent_index[0] = cc_pair.second.index[0];
                                    current_cell->parent_index[1] = cc_pair.second.index[1];
                                }
                                
                            }
                            else
                            {
                                // Add to frontier
                                current_cell->isFrontier = true;
                                
                                // cost = cost_so_far + cost_cell
                                int new_cost = cc_pair.first+ MapObj.cost_map[current_cell->index[0]][current_cell->index[1]].cost;
                                // Add to priority queue
                                std::pair<int,CostMap::GridCell> new_cc_pair = std::make_pair(new_cost,*current_cell);
                                queue.push(new_cc_pair);

                                //Update parent cell of current cell
                                current_cell->parent_index[0] = cc_pair.second.index[0];
                                current_cell->parent_index[1] = cc_pair.second.index[1];
                            }

                            // Check if goal
                            if(current_cell->isGoal)
                                {
                                    logger_->info("Goal Reached!");
                                    break;
                                }

                            MapObj.view_costmap();
                            MapObj.visualise_map();
                            std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        }

                }
                    MapObj.print_map_status();

                    if(current_cell->isGoal)
                    {
                        RetracePathFromGoal(current_cell->index);
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        MapObj.view_costmap();
                        MapObj.visualise_map();
                        break;
                    }

            }

            if(queue.empty()&&!current_cell->isGoal)
            {
                 logger_->error("Run out of grid nodes to search!");
            }
            

            MapObj.is_goal_active = false;
        }
    }


}

bool Dijkstras::CompareCost(int cost,OGMap::GridCell cell)
{

    // Clone the priority queue
    std::priority_queue<std::pair<int, CostMap::GridCell>,
                      std::vector<std::pair<int, CostMap::GridCell>>,
                      CompareClass> queue_clone;

    // True if new cost is lesser than old cost of cell in queue
    bool result = false;
    
    while(!queue.empty())
    {
        std::pair<int,CostMap::GridCell> cc_pair = queue.top();
        if(cc_pair.second.location == cell.location)
        {
            if(cc_pair.first>cost)
            {
                result = true;
                cc_pair.first = cost;
                queue_clone.push(cc_pair);
            }
            else
            {
                queue_clone.push(cc_pair);
            }            
        }
        else
        {
            queue_clone.push(cc_pair);  
        }
        
        queue.pop();
    }
    queue = queue_clone;

    return result;
}

void Dijkstras::RetracePathFromGoal(int goal_index[2])
{
      // Start with last element in the search queue (Goal point)
      OGMap::GridCell* current_cell = MapObj.GetCellbyIndex(goal_index[0],goal_index[1]);
      std::vector<int> cell_coordinates;
      cell_coordinates.push_back(current_cell->location[0]);
      cell_coordinates.push_back(current_cell->location[1]);
      feasible_path_coordinates.push_back(cell_coordinates);
      
      // Mark cell to be on final path for visualisation
      current_cell->isOnPlannerPath = true;

      while(current_cell->parent_index[0]!=-1)
      {
        cell_coordinates.clear();
        current_cell = MapObj.GetCellbyIndex(current_cell->parent_index[0] ,current_cell->parent_index[1]);
        
        // Add it to final path
        cell_coordinates.push_back(current_cell->location[0]);
        cell_coordinates.push_back(current_cell->location[1]);
        feasible_path_coordinates.push_back(cell_coordinates);

        // Mark cell to be on final path for visualisation
         current_cell->isOnPlannerPath = true;

      }

    logger_->info("Number of points on planned path:{}",feasible_path_coordinates.size());

}
