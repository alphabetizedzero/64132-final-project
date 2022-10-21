(define
    (domain robot_world)
    (:requirements :typing)
    
    (:types
        robot-base robot-arm sugar drawer spam - object

        robot-start - zone
        sugar-start sugar-goal robot-grasp-sugar robot-drop-sugar - zone
        spam-start spam-goal robot-grasp-spam robot-drop-spam - zone
    )


        
    (:predicates
        (within ?obj - object ?zone - zone)

        (grasped ?obj - object)
        
        (accessible ?zone - zone)
    )



    (:action pick-up-sugar
        :parameters
            ()
        :precondition
            (and
                (within robot-base robot-grasp-sugar)
                (within sugar sugar-start)
            )
        :effect
            (and
                (grasped sugar)
                (not (within sugar sugar-start))
            )
    )

    (:action put-down-sugar
        :parameters
            ()
        :precondition
            (and
                (grasped sugar)
                (within robot-base robot-drop-sugar)
                (accessible sugar-goal)
            )
        :effect
            (and
                (not (grasped sugar))
                (within sugar sugar-goal)
            )
    )

    (:action pick-up-spam
        :parameters
            ()
        :precondition
            (and
                (within robot-base robot-grasp-spam)
                (within spam spam-start)
            )
        :effect
            (and
                (grasped spam)
                (not (within spam spam-start))
            )
    )

    (:action put-down-spam
        :parameters
            ()
        :precondition
            (and
                (grasped spam)
                (within robot-base robot-drop-spam)
                (accessible spam-goal)
            )
        :effect
            (and
                (not (grasped spam))
                (within sugar spam-goal)
            )
    )

    (:action move-robot-base
        :parameters
            (
                ?from-zone - zone  ; where the robot currently is
                ?to-zone - zone  ; where the robot will end up
            )
        :precondition
            (within robot-base ?from-zone)
        :effect
            (and
                (not (within robot-base ?from-zone))
                (within robot-base ?to-zone)
            )
    )

    (:action open-drawer
        :parameters
            ()
        :precondition
            (and
            	(grasped drawer)
            	(not (accessible spam-goal))  ; Drawer is closed
            )
        :effect
            (accessible spam-goal)
	)

)
