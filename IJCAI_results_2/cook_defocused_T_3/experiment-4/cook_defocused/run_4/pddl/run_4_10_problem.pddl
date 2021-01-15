(define (problem  run_4)

(:domain rapdr)

(:objects
    0.6,0.2,-0.2 0.8,0.0,-0.2 0.6,-0.3,-0.2 0.5,0.0,-0.1 0.5,-0.1,-0.2 0.6,-0.3,0.0 0.6,0.2,0.2 - cartesian
    left_gripper - gripper
    cup cover - obj
    burner1 - burner
)

(:init
    (at table 0.8,0.0,-0.2)
    (at cover 0.5,0.0,-0.1)
    (at cup 0.5,0.0,-0.1)
    (at burner1 0.5,-0.1,-0.2)
    (at left_button 0.6,0.2,-0.2)
    (at right_button 0.6,-0.3,-0.2)
    (at left_gripper 0.6,0.2,0.2)
    (is_visible cup)
    (is_visible cover)
    (touching cup table)
    (touching cover cup)
    (covered cup)
    (pressed left_button)
    (powered_on burner1)
    (pressed right_button)
)

(:goal (cooking cup))

)