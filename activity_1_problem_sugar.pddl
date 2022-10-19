(define (problem sugar_box_move)
	(:domain robot_world)
	(:objects
    	sugar - object
		robot-arm - object
		sugar-grasp - location
		sugar-goal - location
	)

	(:init
		(within sugar sugar-grasp)
		(gripper-empty)
	)
	(:goal 
		(within sugar sugar-goal)
	)
)