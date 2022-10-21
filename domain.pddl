(define
    (domain robot_world)
    (:requirements :typing)
    
    (:types
        robot-base robot-arm sugar drawer spam - object

        robot-start - zone
        sugar-start sugar-goal sugar-grasp - zone
        spam-start spam-goal spam-grasp - zone

         ; robot-start: the zone the robot starts in
         ; sugar-start: the sugar starts in this zone
         ; sugar-grasp: the robot-base must be in this zone to grasp the sugar
         ; sugar-goal: the sugar must be within this zone for the task to be complete
         ; spam-start: the spam starts in this location
         ; spam-grasp: the robot-base must be in this zone to grasp the spam
         ; spam-goal: the spam must be in this zone for the task to be complete
    )


        
    (:predicates
        (within ?obj - object ?zone - zone)

        (grasped ?obj - object)
        
        (accessible ?zone - zone)
    )



    (:action pick-up
        :parameters
            (
                ?obj - object
                ?obj-zone - zone  ; The object is in this zone
                ?robot-zone - zone  ; The robot should be in this zone to pick up the object
            )
        :precondition
            (and
                (within ?obj ?obj-zone)
                (within robot-base ?robot-zone)
                (accessible ?obj-zone)
            )
        :effect
            (and
                (grasped ?obj)
                (not (within ?obj ?obj-zone))
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
                (not (grasped ?obj))
                (within ?obj ?target-zone)
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

    (:action move-robot-arm-to
        :parameters
            (
                ?from-zone - zone  ; where the robot arm currently is
                ?to-zone - zone  ; where the robot arm will end up
            )
        :precondition
            (within robot-arm ?from-zone)
        :effect
            (and
                (not (within robot-arm ?from-zone))
                (within robot-arm ?to-zone)
            )
    )

    (:action open-drawer
        :parameters
            ()
        :precondition
            (and
            	(grasped drawer)
            	(not (accessible ?spam-goal))  ; Drawer is closed
            )
        :effect
            (accessible spam-goal)
	)

)
