#include "breadth_first_search.hpp"

BFS::BFS(OGMap &OGMap) : MapObj(OGMap), planner_alive(true) {

  logger_ = spdlog::get("graph_planning")->clone("bfs_node");

  // Create Motion Model
  motion_model = {{1, 0}, {0, 1},  {-1, 0},  {0, -1}, 
                  {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

  //motion_model = {{1, 0}, {0, 1},  {-1, 0},  {0, -1}};

  // Start Planner
  planner_thread_ = std::thread(&BFS::RunPlanner, this);
}

BFS::~BFS() {

  planner_alive = false;
  planner_thread_.join();
}

void BFS::RunPlanner() {

  logger_->info("Waiting for Goal!");
  while (planner_alive) {
    // Wait to receive start point
    std::unique_lock<std::mutex> goal_lock(goal_mutex_);
    goal_cv.wait_for(goal_lock, std::chrono::milliseconds(2000),
                     [this] { return MapObj.is_goal_active; });

    if (MapObj.is_goal_active) {
      logger_->info("Goal Received!");
    
      // Add start to the queue
      std::vector<int> current_cell_index;
      current_cell_index.push_back(MapObj.start_index[0]);
      current_cell_index.push_back(MapObj.start_index[1]);
      search_queue.push_back(current_cell_index);
      int queue_it = 0;

      //Mark start as a frontier
      MapObj.Map[search_queue[queue_it][0]][search_queue[queue_it][1]].isFrontier = true;
      // End of queue marker
      MapObj.Map[search_queue[queue_it][0]][search_queue[queue_it][1]].parent_index[0] = -1;
      MapObj.Map[search_queue[queue_it][0]][search_queue[queue_it][1]].parent_index[1] = -1;
      MapObj.visualise_map();
      while(planner_alive){

          // Remove from frontier add to explored
           OGMap::GridCell* current_cell = MapObj.GetCellbyIndex(search_queue[queue_it][0],search_queue[queue_it][1]);
           current_cell->isFrontier = false;
           current_cell->isExplored = true;

            MapObj.visualise_map();

           for(int i=0;i<motion_model.size();i++)
           {
               // Check if point is not explored and free
                current_cell = MapObj.GetCellbyIndex(search_queue[queue_it][0]+motion_model[i][0],search_queue[queue_it][1]+motion_model[i][1]);
                if(!current_cell->isExplored && !current_cell->isFrontier && current_cell->value==OGMap::FREE)
                {
                    // Add to frontier
                    current_cell->isFrontier = true;

                    // Add to search queue
                    current_cell_index.clear();
                    current_cell_index.push_back(search_queue[queue_it][0]+motion_model[i][0]);
                    current_cell_index.push_back(search_queue[queue_it][1]+motion_model[i][1]);
                    search_queue.push_back(current_cell_index);

                    //Update parent cell of current cell
                    current_cell->parent_index[0] = search_queue[queue_it][0];
                    current_cell->parent_index[1] = search_queue[queue_it][1];

                    // Check if goal
                    if(current_cell->isGoal)
                        {
                             logger_->info("Goal Reached!");
                             break;
                        }

                    MapObj.visualise_map();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }

           }

              if(current_cell->isGoal)
            {
                BuildPathFromQueue();
                MapObj.visualise_map();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                break;
            }

             if(queue_it > search_queue.size()-1)
            {
                logger_->error("Run out of grid nodes to search!");
                break;
            }

         queue_it++;
      }
      

      MapObj.is_goal_active = false;
    }
  }
}


void BFS::BuildPathFromQueue()
{
    
    if(search_queue.size()>1)
    {
      // Start with last element in the search queue (Goal point)
      int last_index = search_queue.size()-1;
      OGMap::GridCell* current_cell = MapObj.GetCellbyIndex(search_queue[last_index][0],search_queue[last_index][1]);
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

    }

    logger_->info("Number of points on planned path:{}",feasible_path_coordinates.size());

}