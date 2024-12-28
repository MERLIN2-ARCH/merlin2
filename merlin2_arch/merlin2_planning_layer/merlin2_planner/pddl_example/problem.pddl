(define (problem demo_prb)
    (:domain demo)
    (:objects

        ; locations
        kitchen bedroom livingroom bathroom entrance - location

        ; people
       Miguel - person
    )
    (:init
        ; the robot at start is in the entrance of the house
        (robot_at entrance)
        (person_at miguel livingroom)
    )
    (:goal
        (attended miguel)
    )
)