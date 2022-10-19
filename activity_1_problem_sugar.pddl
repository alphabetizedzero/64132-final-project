(define (problem sugar_box_move)
	(:domain robot_world)
	(:objects
    	sugar_box - sugar_box
		robot_arm - robot
		burner - location
		counter - location
	)

	(:init
		(on sugar_box burner)
		(robot_arm_empty)
	)
	(:goal 
		(on sugar_box counter)
		(robot_location sugar_goal_position)
		(robot_arm_empty)
	)
)