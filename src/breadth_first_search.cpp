#include "breadth_first_search.hpp"

BFS::BFS(float start[], OGMap &OGMap) : MapObj(OGMap), planner_alive(true) {

  logger_ = spdlog::get("graph_planning")->clone("bfs_node");

  // Create Motion Model
  motion_model = {{1, 0}, {0, 1},  {-1, 0},  {0, -1},
                  {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

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
    /*
      // Add start to the queue
      std::vector<int> current_cell;
      current_cell.push_back(MapObj.Map[0]);
      current_cell.push_back(MapObj.Map[0]);
      search_queue.push_back(current_cell);
      int queue_it = 0;

      while(1){







      }
      */

      MapObj.is_goal_active = false;
    }
  }
}
