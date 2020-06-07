#include "depth_first_search.hpp"

DFS::DFS(OGMap &OGMap) : MapObj(OGMap), planner_alive(true) {

  logger_ = spdlog::get("graph_planning")->clone("dfs_node");

  // Create Motion Model
  motion_model = {{1, 0}, {0, 1},  {-1, 0},  {0, -1}, 
                  {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

  motion_model = {{1, 0}, {0, 1},  {-1, 0},  {0, -1}};

  // Start Planner
  planner_thread_ = std::thread(&DFS::RunPlanner, this);
}

DFS::~DFS() {

  planner_alive = false;
  planner_thread_.join();
}

void DFS::RunPlanner() {

  logger_->info("Waiting for Goal!");
  while (planner_alive) {
    // Wait to receive start point
    std::unique_lock<std::mutex> goal_lock(goal_mutex_);
    goal_cv.wait_for(goal_lock, std::chrono::milliseconds(2000),
                     [this] { return MapObj.is_goal_active; });

    if (MapObj.is_goal_active) {
      logger_->info("Goal Received!");
    
      // Add start to the stack
      std::vector<int> current_cell_index;
      current_cell_index.push_back(MapObj.start_index[0]);
      current_cell_index.push_back(MapObj.start_index[1]);
      search_stack.push(current_cell_index);

      // End of queue marker
      MapObj.Map[current_cell_index[0]][current_cell_index[1]].parent_index[0] = -1;
      MapObj.Map[current_cell_index[0]][current_cell_index[1]].parent_index[1] = -1;
      MapObj.visualise_map();
      while(planner_alive&& (!search_stack.empty())){

          // Get top element from stack
          current_cell_index.clear();
          current_cell_index = search_stack.top();

          // Add to frontier
           OGMap::GridCell* current_cell = MapObj.GetCellbyIndex(current_cell_index[0],current_cell_index[1]);
           current_cell->isFrontier = true;

            MapObj.visualise_map();
           
           int i;
           for(i=0;i<motion_model.size();i++)
           {
               // Check if point is not explored and free
                current_cell = MapObj.GetCellbyIndex(current_cell_index[0]+motion_model[i][0],current_cell_index[1]+motion_model[i][1]);
                if(!current_cell->isExplored && !current_cell->isFrontier && current_cell->value==OGMap::FREE)
                {
                    // Add to frontier
                    current_cell->isFrontier = true;

                    // Add to search stack
                    current_cell_index.clear();
                    current_cell_index.push_back(current_cell_index[0]+motion_model[i][0]);
                    current_cell_index.push_back(current_cell_index[1]+motion_model[i][1]);
                    search_stack.push(current_cell_index);

                    //Update parent cell of current cell
                    current_cell->parent_index[0] = current_cell_index[0];
                    current_cell->parent_index[1] = current_cell_index[1];

                    // Check if goal
                    if(current_cell->isGoal)
                        {
                             logger_->info("Goal Reached!");
                             break;
                        }

                    MapObj.visualise_map();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    // Go back to top of stack
                    break;
                }

           }
            MapObj.print_map_status();

              if(current_cell->isGoal)
            {
                BuildPath();
                MapObj.visualise_map();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                break;
            }
            // If we have finished exploring current cell, add it to explored and pop it from stack
            if(!current_cell->isGoal&&i==motion_model.size())
            {
                search_stack.pop();
                current_cell->isExplored = true;
                 current_cell->isFrontier = false;
                MapObj.visualise_map();
            }

      }

      if(search_stack.empty())
      {
        logger_->error("Run out of grid nodes to search!");
      }
      

      MapObj.is_goal_active = false;
    }
  }
}


void DFS::BuildPath()
{
    
    if(!search_stack.empty())
    {
      // Top element in search stack is (Goal point)
      std::vector<int> goal_cell_index = search_stack.top();
      OGMap::GridCell* current_cell = MapObj.GetCellbyIndex(goal_cell_index[0],goal_cell_index[1]);
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
          logger_->info("{}",current_cell->parent_index[0]);
      }

    }
    else
    {
       logger_->error("Build path got empty search stack!");
    }
    

    logger_->info("Number of points on planned path:{}",feasible_path_coordinates.size());

}