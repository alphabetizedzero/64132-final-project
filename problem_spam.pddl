(define (problem spam_box_move)
	(:domain robot_world)
	(:objects
    	spam - object
		robot-arm - object
		spam-grasp - location
		spam-goal - location
		drawer - object
	)

	(:init
		(gripper-empty)
	)
	(:goal 
		(within spam spam-goal)
	)
)
