# 64132-final-project


## Activity Planning

### What's where

 * problem.pddl: PDDL problem definition
 * domain.pddl: PDDL domain definition
 * bfs_planner.py: BFS planner for high level activity planning

### Explanation

To break down the problem into managable parts, we first have some assumptions:
* The robot is given the locations of the sugar box, spam box, and the goal.
* There will always be a sugar box, a spam box, goal zones, and a robot.
* The robot's sensors are accurate, meaning that the action that it is told to perform will be completed 100% of the time.
* The motion of the robot can be abstracted away in the activity plan, and the robot can be commanded to 

Next, we determine the predicates needed to describe the states of the world in a domain file.

The target objects (sugar box and spam box) can be within a zone (starting or goal) or grasped. The robot's gripper can be full or empty. The zones can have different accessiblity.

The actions that can be taken would be the robot picking up an object, putting down an object, moving the robot to positions, and opening the drawer.

The problems have separate problem files. Each will be solved independent of the other. 

Because the number of possible states is relatively low for the current implemented domain and problems, there is no need for a heuristic. The planner implemented is a Breadth-First Search on a planning graph for both the sugar and spam problem.


## Sample Based Motion Planning

For this part of the project we decided to implement motion planning using RRT.

### What's where

 * robot_commands.py: implementations of our actions from the BFS planner
 * rrt_draft.py: implementation of the RRT algorithm for arm motion planning
 * simulation.py: our simulation environment and execution engine

### Explanation

Our BFS planner from part 1 returns a list of actions to take in order to accomplish the goals defined in the problem.pddl file. We are working on creating general implementations of all of these actions in order to command the robot to complete the tasks in the sim. Our execution engine then takes the list of actions and calls their corresponding implementation and passes any relevent parameters to command the robot in the simulation.

The BFS planner outputs a list of actions some of which are motion actions between two points. We will maintain a hardcoded map between the names of these points and their actual coordinates in the world frame in order. We will also assume that when the end effector is within a minimum distance of objects and the robot takes an action whose intent is to grasp the object, that the object is grasped without error. We will accomplish this by welding the objects to the robot end effector while they are "grasped".

For the actions that involve moving the robot base between different points, we make the assumption that the floor is clear with no obstacles that could cause a collision. This assumption allows us to move the base in straight line paths directly between points without more complex motion planning.

Moving the arm is more complicated because there are objects that could be in the way of the arm. For these actions, we will use RRT to determine a path between points that does not intersect any objects in the world.