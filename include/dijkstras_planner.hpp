// created by indraneel on 16/06/20

#ifndef DIJKSTRAS_PLANNER_HPP
#define DIJKSTRAS_PLANNER_HPP

#include <iostream>
#include <vector>
#include <queue>

#include <spdlog/spdlog.h>

#include "cost_map.hpp"

class Dijkstras {

public:
    Dijkstras(CostMap &CostMap);
    ~Dijkstras();

    /** @brief : Compare class for the priority queue */
    class CompareClass {
        public:
            bool operator() (std::pair<int, CostMap::MapCell> a, std::pair<int, CostMap::MapCell> b) {
                if (a.first < b.first)
                    return true;
                return false;
        }
    };


    std::vector<std::vector<int>> feasible_path_coordinates;
private:
    std::vector<std::vector<int>> motion_model;
    std::shared_ptr<spdlog::logger> logger_;
    CostMap &MapObj;

    bool planner_alive;
    std::thread planner_thread_;
    std::priority_queue <std::pair<int, CostMap::MapCell>,std::vector<std::pair<int, CostMap::MapCell>>,CompareClass> queue; 

};
#endif