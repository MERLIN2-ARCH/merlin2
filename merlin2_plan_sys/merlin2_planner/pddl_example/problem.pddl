(define (problem merlin2_prb)
(:domain merlin2)
(:objects
    rb1 - robot
    wp1 - wp
    wp2 - wp
)
(:init
    (robot_at rb1 wp1)
)
(:goal
    (wp_checked rb1 wp2)
)
)