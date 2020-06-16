// created by indraneel on 17/06/2020

#include "dijkstras_planner.hpp"

Dijkstras::Dijkstras(CostMap &CostMap) : MapObj(CostMap), planner_alive(true) {

    logger_ = spdlog::get("graph_planning")->clone("dijkstras_node");

     motion_model = {{1, 0}, {0, 1},  {-1, 0},  {0, -1}};

    // Testing priority queue
    int i = 0;
    while(i<5)
    {
        std::pair<int,CostMap::MapCell> temp_pair = std::make_pair(10*i,MapObj.cost_map[i][0]);
        queue.push(temp_pair);
        i++;
    }

    while(!queue.empty())
    {
        std::pair<int,CostMap::MapCell> res = queue.top();
        logger_->info("{}",res.first);
        queue.pop();;
    }

}

Dijkstras::~Dijkstras()
{}
