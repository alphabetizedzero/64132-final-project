(define
    (domain robot_world)
    (:requirements :typing)
    
    (:types
        robot-base robot-arm sugar drawer spam - object

        robot-start - zone
        sugar-start sugar-goal robot-base-grasp-sugar robot-base-drop-sugar - zone
        spam-start spam-goal robot-base-grasp-spam robot-base-drop-spam - zone
    )


        
    (:predicates
        (within ?obj - object ?zone - zone)

        (grasped ?obj - object)
        (gripper-empty)
        
        (accessible ?zone - zone)
    )
    

    (:action pick-up-sugar
        :parameters
            ()
        :precondition
            (and
                (gripper-empty)
                (within robot-base robot-base-grasp-sugar)
                (within sugar sugar-start)
            )
        :effect
            (and
                (grasped sugar)
                (not (gripper-empty))
                (not (within sugar sugar-start))
            )
    )

    (:action put-down-sugar
        :parameters
            ()
        :precondition
            (and
                (grasped sugar)
                (not (gripper-empty))
                (within robot-base robot-base-drop-sugar)
                (accessible sugar-goal)
            )
        :effect
            (and
                (not (grasped sugar))
                (gripper-empty)
                (within sugar sugar-goal)
            )
    )

    (:action pick-up-spam
        :parameters
            ()
        :precondition
            (and
                (gripper-empty)
                (within robot-base robot-base-grasp-spam)
                (within spam spam-start)
            )
        :effect
            (and
                (grasped spam)
                (not (gripper-empty))
                (not (within spam spam-start))
            )
    )

    (:action put-down-spam
        :parameters
            ()
        :precondition
            (and
                (grasped spam)
                (not (gripper-empty))
                (within robot-base robot-base-drop-spam)
                (accessible spam-goal)
            )
        :effect
            (and
                (not (grasped spam))
                (gripper-empty)
                (within spam spam-goal)
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
                (gripper-empty)
                (not (accessible spam-goal))  ; Drawer is closed
                (within robot-base robot-base-drop-spam)
                ; TODO: In order to have (grasped drawer) precondition, we need a (grasp ?obj) action which requires some way of knowing
                ; if we're in range of the ?obj. Alternatively, we could have a specific grasp-drawer action
            )
        :effect
            (accessible spam-goal)
	)

    (:action close-drawer
        :parameters
            ()
        :precondition
            (and
                (gripper-empty)
                (accessible spam-goal)  ; Drawer is open
                (within robot-base robot-base-drop-spam)
            )
        :effect
            (not (accessible spam-goal))
	)

)
