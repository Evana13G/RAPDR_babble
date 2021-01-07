(define (domain rapdr)

(:requirements :strips :typing :negative-preconditions)

(:types
    entity location - object
    obj gripper burner - entity
    cartesian - location
)

(:predicates
    (at ?e - entity ?loc - cartesian)
    (touching ?e - entity ?e - entity)
    (is_visible ?e - entity)
    (covered ?e - entity)
    (on_burner ?e - entity ?e - entity)
    (cooking ?e - entity)
    (food_cooked)
    (powered_on ?e - entity)
    (pressed ?e - entity)
    (on ?e - entity ?e - entity)
    (prepped ?e - entity)
    (shaken ?e - entity)
    (ingredients_added ?e - entity)
)

(:action push
    :parameters (?g - gripper ?loc0 - cartesian ?o - obj )
    :precondition (at ?o ?loc0)
    :effect (not (at ?o ?loc0))
)

(:action shake
    :parameters (?g - gripper ?o - obj )
    :precondition (not (covered ?o))
    :effect (shaken ?o)
)

(:action uncover_obj
    :parameters (?g - gripper ?o1 - obj ?loc1 - cartesian ?o2 - obj )
    :precondition (and
        (covered ?o2)
        (at ?o2 ?loc1)
        (not (at ?o1 ?loc1)))
    :effect (not (covered ?o2))
)

(:action cover_obj
    :parameters (?g - gripper ?o1 - obj ?loc1 - cartesian ?o2 - obj )
    :precondition (and
        (not (covered ?o2))
        (at ?o2 ?loc1)
        (not (at ?o1 ?loc1)))
    :effect (covered ?o2)
)

(:action place_on_burner
    :parameters (?g - gripper ?o - obj ?b - burner )
    :precondition (and
        (not (covered ?o))
        (prepped ?o))
    :effect (on_burner ?o ?b)
)

(:action cook
    :parameters (?g - gripper ?o - obj ?b - burner )
    :precondition (and
        (on_burner ?o ?b)
        (covered ?o))
    :effect (cooking ?o)
)

(:action check_food
    :parameters (?g - gripper ?o - obj )
    :precondition (cooking ?o)
    :effect (food_cooked)
)

(:action prep_food
    :parameters (?g - gripper ?o - obj )
    :precondition (and
        (not (covered ?o))
        (shaken ?o))
    :effect (prepped ?o)
)

(:action add_ingredients
    :parameters (?g - gripper ?o - obj )
    :precondition (not (covered ?o))
    :effect (ingredients_added ?o)
)

(:action push-orientation:right
    :parameters (?g - gripper ?loc0 - cartesian ?o - obj )
    :precondition (at ?o ?loc0)
    :effect (not (at ?o ?loc0))
)

(:action push-orientation:right_V2
    :parameters (?g - gripper ?loc0 - cartesian ?o - obj )
    :precondition (at ?o ?loc0)
    :effect (not (at ?o ?loc0))
)

)