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
    - [ ] Adapt search queue to priority queue
    - [ ] track movement costs
    - [ ] Visualise the frontier,explored nodes and final path
    - [ ] Introduce movement cost variable
    
    

Edit the document or wiki page and use the - [ ] and - [x] syntax to update your task list.
