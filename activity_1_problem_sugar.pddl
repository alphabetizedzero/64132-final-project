(define (problem sugar_box_move)
	(:domain robot_world)

	(:init
		(within robot-base robot-start)
		(within sugar sugar-start)
		(within spam spam-start)

		(gripper-empty)

		(accessible sugar-start)
		(accessible sugar-goal)
		(accessible spam-start)
	)
	(:goal 
		(and
			(within sugar sugar-goal)
			(within spam spam-goal)
			(not (accessible spam-goal))  ; End with the drawer closed
		)
	)
)