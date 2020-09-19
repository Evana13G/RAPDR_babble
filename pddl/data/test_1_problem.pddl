(define (problem  test)

(:domain rapdr)

(:objects
    0.6,0.2,1.2 0.5,0.0,-0.1 0.8,0.0,-0.9 0.6,-0.3,1.0 - cartesian
    left_gripper right_gripper - gripper
    table cover cup - obj
)

(:init
    (at cover 0.5,0.0,-0.1)
    (at cup 0.5,0.0,-0.1)
    (at left_gripper 0.6,0.2,1.2)
    (at right_gripper 0.6,-0.3,1.0)
    (at table 0.8,0.0,-0.9)
    (is_visible cup)
    (is_visible cover)
    (touching cover cup)
)

(:goal (grasped cover))

)