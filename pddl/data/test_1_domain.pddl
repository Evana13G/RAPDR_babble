(define (domain rapdr)

(:requirements :strips :typing :fluents :disjunctive-preconditions)

(:types
    entity location - object
    obj gripper - entity
    cartesian - location
)

(:predicates
    (at ?e - entity ?loc - cartesian)
    (touching ?e - entity ?e - entity)
    (is_visible ?e - entity)
    (grasped ?o - obj)
)

(:action push
    :parameters (?g - gripper ?loc0 - cartesian ?o - obj ?loc1 - cartesian )
    :precondition (at ?o ?loc0)
    :effect (and
        (at ?o ?loc1)
        (not (at ?o ?loc0)))
)

(:action grasp
    :parameters (?g - gripper ?o - obj )
    :precondition (and)
    :effect (grasped ?o)
)

(:action shake
    :parameters (?g - gripper ?o - obj )
    :precondition (and)
    :effect (and)
)

(:action press
    :parameters (?g - gripper ?o - obj )
    :precondition (and)
    :effect (and)
)

(:action drop
    :parameters (?g - gripper ?o - obj )
    :precondition (and)
    :effect (and)
)

)