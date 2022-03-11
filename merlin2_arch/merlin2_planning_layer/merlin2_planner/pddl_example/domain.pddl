(define (domain demo)

    (:requirements :typing :negative-preconditions :durative-actions)

    (:types
        location ; service areas, points of interest, navigation goals
        object ; objects to be manipulated by the robot
        person ; a human being who needs to be conquered
        sentence ; a sentence to express world domination joy
        )

    (:predicates
        ; the robot is at location ?l
        (robot_at ?l - location)

        ; the person is attended
        (attended ?p -person)

        ; the person is at location ?l
        (person_at ?p - person ?l - location)

    )

    ; navigation action
    (:durative-action navigation
        :parameters (?source ?destination - location)
        :duration (= ?duration 10)
        :condition (and
            ;(at end (not (robot_at ?destination)))
            (at end (robot_at ?source))
        )
        :effect (and
            (at end (not (robot_at ?source)))
            (at end (robot_at ?destination))
        )
    )

    ; hi_navigation action
    (:durative-action hi_navigation
        :parameters (?source_p ?source_r - location ?per - person)
        :duration (= ?duration 10)
        :condition (and (at start (robot_at ?source_p)) (at start (person_at ?per ?source_p)))
        :effect (and
            (at end (attended ?per))
            (at end (not (robot_at ?source_p)))
        )
    )
)