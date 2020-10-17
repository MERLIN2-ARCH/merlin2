(define (domain merlin2)
(:requirements :typing :negative-preconditions :durative-actions)
(:types
    robot
    wp
)
(:predicates
    (robot_at ?r0 - robot ?w1 - wp)
    (wp_checked ?r0 - robot ?w1 - wp)
)
(:durative-action navigation
    :parameters ( ?r - robot ?s - wp ?d - wp)
    :duration (= ?duration 10)
    :condition (and
        (at start (robot_at ?r ?s))
    )
    :effect (and
        (at start (not (robot_at ?r ?s)))
        (at end (robot_at ?r ?d))
    )
)
(:durative-action check_wp
    :parameters ( ?r - robot ?s - wp)
    :duration (= ?duration 10)
    :condition (and
        (at start (robot_at ?r ?s))
    )
    :effect (and
        (at end (wp_checked ?r ?s))
    )
)
)