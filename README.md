# 64132-final-project


Activity Planning

To break down the problem into managable parts, we first create some assumptions:
* The robot is given the locations of the sugar box, spam box, and the goal.
* There will always be a sugar box, a spam box, goal zones, and a robot.
* The robot's sensors are accurate, meaning that the action that it is told to perform will be completed 100% of the time.
* The motion of the robot can be abstracted away in the activity plan, and the robot can be commanded to 

Next, we determine the predicates needed to describe the states of the world in a domain file.

The target objects (sugar box and spam box) can be within a zone (starting or goal) or grasped. The robot's gripper can be full or empty. The zones can have different accessiblity.

The actions that can be taken would be the robot picking up an object, putting down an object, moving the robot to positions, and opening the drawer.

The problems have separate problem files. Each will be solved independent of the other. 

Because the number of possible states is relatively low for the current implemented domain and problems, there is no need for a heuristic. The planner implemented is a Breadth-First Search on a planning graph for both the sugar and spam problem.
