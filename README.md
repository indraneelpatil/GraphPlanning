# GraphPlanning
Global planning for robotics based on graph search technique

### TODO
+ Accepting a new goal in rviz
    - [x] Create new subscriber with check for current goal processing
    - [x] Constraint check for the goal with dimensions of map
    - [x] Round up the goal to cell of the map
    - [x] Visualise the goal in rviz
    - [x] Update the Map object with the new goal (wipe off any old goals as well)
+ Breadth First Search Planner
    - [X] Write Motion Model
    - [X] Planning thread wait to receive goal
    - [X] Iterate queue until goal cell is found
    - [X] Build final path
    - [X] Visualise frontier, explored nodes and final path
    - [X] Add progress indicator during planning and performance metrics
    - [X] Clear Map function
    - [X] Add obstacles
    - [X] Is diagonal exploration possible?
 + Depth First Search Planner
    - [X] Create stack data structure
    - [X] Adapt breadth first search to depth first search
 + 2D Cost Map
    - [X] CostMap class as derived class of OGMap class
    - [X] Initialise all costs to default values
    - [X] Inflate OG Map obstacles in the cost map
    - [X] Visualise Cost Map using marker array
    - [X] what should be default cost value of cost map cells
 + Dijkstra's Planner
    - [X] Adapt search queue to priority queue
    - [X] track movement costs
    - [X] Visualise the frontier,explored nodes and final path
    - [X] Introduce movement cost variable (No such variable needed ??? -> taken care of by COST_NEUTRAL)
    - [X] Trade off between path length and proximity to obstacles (COST_NEUTRAL vs cost_scaling_factor)
 + Greedy Best First Search Planner
    - [ ] Priority Queue returns cells closest to the goal
 + A Star Planner
    - [X] Introduce the manhattan distance (from goal) heuristic
    - [X] Weight of the heuristic variable (Optimality condition : heuristic <true distance)
    - [X] heuristic_weight variable -> trade off between Dijkstras and Greedy Best first search
    - [X] Implement A Star with a compile time macro switch in Dijkstras
    
 

### Depth First Search vs Breadth First Search
[![Watch the video](https://img.youtube.com/vi/NP1IigrCXcg/hqdefault.jpg)](https://www.youtube.com/watch?v=NP1IigrCXcg)  

### A* Planner vs Dijkstras Planner
[![Watch the video](https://img.youtube.com/vi/s5UPQj2Uy40/hqdefault.jpg)](https://www.youtube.com/watch?v=s5UPQj2Uy40)   
    
    

