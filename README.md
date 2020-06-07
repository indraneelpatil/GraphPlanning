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
    - [X] Iterate until goal cell is found
    - [X] Build final path
    - [X] Visualise frontier, explored nodes and final path
    - [X] Add progress indicator during planning and performance metrics
    - [X] Clear Map function
    - [X] Add obstacles
    - [X] Is diagonal exploration possible?
 + Depth First Search Planner
    - [ ] Create stack data structure
    - [ ] Adapt breadth first search to depth first search

Edit the document or wiki page and use the - [ ] and - [x] syntax to update your task list.
