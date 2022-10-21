(define
    (domain robot_world)
    (:requirements :typing)
    
    (:types
        robot-base robot-arm sugar location spam drawer- object
        sugar-grasp sugar-goal spam-grasp spam-goal - zone
         ; sugar-grasp: the robot-base must be in this zone to grasp the sugar
         ; sugar-goal: the sugar must be within this zone for the task to be complete
    )


        
    (:predicates
        (within ?obj - object ?zone - zone)

        (grasped ?obj - object)
        
        (accessible ?zone - zone)

        (drawer-open)
    )



    (:action pick-up
        :parameters
            (
                ?obj - object
                ?robot-zone - zone  ; The robot should be in this zone to pick up the object
            )
        :precondition
            (and
                (within robot-base ?robot-zone)
            )
        :effect
            (and
                (grasped ?obj)
            )
    )

    (:action put-down
        :parameters
            (
                ?obj - object
                ?target-zone - zone  ; The object should be put down in this zone
                ?robot-zone - zone  ; The robot should be in this zone to put down the object
            )
        :precondition
            (and
                (grasped ?obj)
                (within robot-base ?robot-zone)
                (accessible ?target-zone)
            )
        :effect
            (and
                (within ?obj ?target-zone)
            )
    )

    (:action move-robot-base-to
        :parameters
            (
                ?zone - zone
            )
        :precondition
            ()
        :effect
            (and
                (within robot-base ?zone)
            )
    )

    (:action move-robot-arm-to
        :parameters
            (
                ?zone - zone
            )
        :precondition
            ()
        :effect
            (and
                (within robot-arm ?zone)
            )
    )
    (:action open-drawer
        :parameters
            ()
        :precondition
            (and
            	(grasped drawer)
            	(not (drawer-open))
            )
        :effect
            (and
                (drawer-open)
                (accessible spam-goal)
            )
	)

)
