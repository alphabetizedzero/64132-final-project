(define
    (domain robot_world)
    (:requirements :typing)
    
    (:types
        robot-base robot-arm sugar location - object
        sugar-grasp sugar-goal - zone
         ; sugar-grasp: the robot-base must be in this zone to grasp the sugar
         ; sugar-goal: the sugar must be within this zone for the task to be complete
    )


        
    (:predicates
        (within ?obj - object ?z - zone)

        (grasped ?obj - object)

        (drawer-open)
        (gripper-empty)
    )



    (:action pick-up
        :parameters
            (
                ; TODO: remove ?s before parameter names
                ?obj - object
                ?robot-zone - zone  ; The robot should be in this zone to pick up the object
            )
        :precondition
            (and
                (gripper-empty)
                (within robot-base ?robot-zone)
            )
        :effect
            (and
                (not (gripper-empty))
                (grasped ?obj - object)
            )
    )

    (:action put-down
        :parameters
            (
                ?target-zone - zone  ; The object should be put down in this zone
                ?robot-zone - zone  ; The robot should be in this zone to put down the object
            )
        :precondition
            (and
                (not (gripper-empty))
                (within robot-base ?robot-zone)
            )
        :effect
            (and
                (gripper-empty)
            )
    )

    (:action move-robot-base-to
        :parameters
            (
                ?z - zone
            )
        :precondition
            ()
        :effect
            (and
                (within robot-base ?z)
            )
    )

    (:action move-robot-arm-to
        :parameters
            (
                ?z - zone
            )
        :precondition
            ()
        :effect
            (and
                (within robot-arm ?z)
            )
    )


)