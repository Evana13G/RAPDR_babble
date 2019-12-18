(define (domain rapdr)

(:requirements :strips :typing :fluents :disjunctive-preconditions)

(:types
    location obj gripper button - object
    waypoint - location
)

(:predicates
    (gripper_at ?g - gripper ?wp - waypoint)
    (obj_at ?o - obj ?wp - waypoint)
    (button_at ?b - button ?wp - waypoint)
    (pressed ?b - button)
    (is_visible ?o - obj)
    (obtained ?o - obj)
)

(:action obtain_object
    :parameters (?g - gripper ?loc0 - waypoint ?o - obj ?loc1 - waypoint )
    :precondition (obj_at ?o ?loc1 )
    :effect (and
        (obj_at ?o ?loc0 )
        (obtained ?o ))
)

(:action press_button
    :parameters (?g - gripper ?loc0 - waypoint ?b - button ?loc1 - waypoint )
    :precondition (and
        (gripper_at ?g ?loc0 )
        (button_at ?b ?loc1 ))
    :effect (pressed ?b )
)

(:action action_attempt_1_trial2_seg6
    :parameters (?g - gripper ?b - button )
    :precondition (and)
    :effect (pressed ?b )
)

(:action action_attempt_1_trial3_seg2
    :parameters (?g - gripper ?o - obj ?obj0 - obj )
    :precondition (and)
    :effect (pressed ?obj0 )
)

(:action action_attempt_1_trial3_seg3
    :parameters (?g - gripper ?o - obj ?obj0 - obj )
    :precondition (and)
    :effect (is_visible ?obj0 )
)

(:action action_attempt_1_trial4_seg6
    :parameters (?g - gripper ?b - button ?obj0 - obj )
    :precondition (and)
    :effect (and
        (pressed ?b )
        (is_visible ?obj0 ))
)

)