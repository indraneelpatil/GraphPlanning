cmake_minimum_required(VERSION 2.8.3)
project(graph_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BOOST_CXX_FLAGS} -std=c++14" )
find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        geometry_msgs
        roscpp
        rosconsole
        std_msgs
        tf
        rospy
        roslint
        nav_msgs
        rosbag
        genmsg
        message_generation
        nav_2_0_common_msgs
        gor_map_server
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(spdlog REQUIRED
        HINTS /usr/local/lib)
find_package(Threads REQUIRED)

include_directories(
                    include   
                    ${catkin_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES marker_feedback
        CATKIN_DEPENDS roscpp tf geometry_msgs rosbag nav_msgs gor_map_server realtime_tools
)

add_library(maps SHARED
        src/occupancy_grid_map.cpp
        src/cost_map.cpp
)


set_target_properties(maps PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(maps
                ${CMAKE_THREAD_LIBS_INIT}
                spdlog
                ${catkin_LIBRARIES}
                )

add_library(planners SHARED
src/breadth_first_search.cpp
src/depth_first_search.cpp
src/dijkstras_planner.cpp
)


set_target_properties(planners PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(planners
        ${CMAKE_THREAD_LIBS_INIT}
        spdlog
        ${catkin_LIBRARIES}
        )

add_executable(planner_main src/planner_main.cpp)
target_link_libraries(planner_main 
                        maps
                        planners
                        #lift_up_tree_nodes
                        boost_system
                        spdlog
                # lift_controller
                ${CMAKE_THREAD_LIBS_INIT}
                ${catkin_LIBRARIES}
                )
set_target_properties(planner_main PROPERTIES LINKER_LANGUAGE CXX)

add_executable(planner_main_cost src/planner_main_cost.cpp)
target_link_libraries(planner_main_cost
                        maps
                        planners
                        #lift_up_tree_nodes
                        boost_system
                        spdlog
                # lift_controller
                ${CMAKE_THREAD_LIBS_INIT}
                ${catkin_LIBRARIES}
                )
set_target_properties(planner_main_cost PROPERTIES LINKER_LANGUAGE CXX)