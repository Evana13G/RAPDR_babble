#!/usr/bin/env python

import rospy

from std_msgs.msg import Empty

from pddl.srv import *

pddlInstatiations = rospy.ServiceProxy('get_pddl_instatiations_srv', GetActionPDDLBindingSrv)

def check_pddl_effects(req):
    actionName = req.actionName
    args = req.args
    actual_pre = req.preconditions
    actual_effects = req.effects
    expectatation = pddlInstatiations(actionName, args).pddlBindings
    exp_pre = expectatation.preconditions
    exp_effects = expectatation.effects

    # These need to be the same... for APV. For checking effects, you need the expected 
    # to be a subset of actual 
    CONDITION_loc = loc_effects_met(actual_pre, actual_effects, exp_pre, exp_effects)
    CONDITION_nonLoc = nonLoc_effects_met(actual_pre, actual_effects, exp_pre, exp_effects)

    return (CONDITION_loc and CONDITION_nonLoc) == True

def novel_effect(req):
    actionName = req.actionName
    args = req.args
    actual_pre = req.preconditions
    actual_effects = req.effects
    expectatation = pddlInstatiations(actionName, args).pddlBindings.effects

    novelty = False
    new_effects = []

    ## LOC 
    # locChanging_actual = detect_loc_changing_objects(w_negation)
    # locChanging_expected = detect_loc_changing_objects(expectatation)

    ## Non Loc
    w_negation = generate_effects_negations(actual_pre, actual_effects)

    effects = [x for x in w_negation if 'at' not in x]
    expected_effects = [x for x in expectatation if 'at' not in x] 
    new_effects = [x for x in effects if x not in actual_pre]
    
    if all(x in expected_effects for x in new_effects) == False:
        return True, new_effects

    return (new_effects != []), new_effects

#### HELPER FUNCTIONS #########################################################################
def loc_effects_met(pre, eff, exp_pre, exp_eff):
    w_negation = generate_effects_negations(pre, eff)
    locChanging_actual = detect_loc_changing_objects(w_negation)
    locChanging_expected = detect_loc_changing_objects(exp_eff)

    # It wont always explicitly negate... so you would need to call the negation function
    # and if it does explicitly negate, it might not specify a new location... 
    # whereas in the actual pre/eff, there should be an actual location specified
    # The detection function checks to see if there 
    if locChanging_expected == []:
        locChanging_expected = detect_loc_changing_objects(exp_eff + exp_pre)
        # If you add them, and they have the negation, than you should be good..

    if all(x in locChanging_actual for x in locChanging_expected):
        return True
    return False

def nonLoc_effects_met(pre, eff, exp_pre, exp_eff): 
    w_negation = generate_effects_negations(pre, eff)

    effects = [x for x in w_negation if 'at' not in x]
    expected_effects = [x for x in exp_eff if 'at' not in x] 
    
    if all(x in effects for x in expected_effects):
        return True
    return False  

# This assumes that there is a negation of its original, and that there is a new
# Loc, which might not always be the case in the case of the KB representation.. 

def detect_loc_changing_objects(predicates):
    effects = [x for x in predicates if 'at' in x]
    negativeLoc_objs = [x[9:].split()[0] for x in effects if '(not ' in x]
    positiveLoc_objs = [x[4:].split()[0] for x in effects if '(not ' not in x]
    loc_changing_objects = [x for x in positiveLoc_objs if x in negativeLoc_objs]
    return loc_changing_objects 

def generate_effects_negations(preconditions, effects):
    for pre in preconditions:
        if pre not in effects:
            effects.append('(not ' + pre + ')')
    return effects



###########################################################################
def main():
    rospy.init_node("pddl_checker_node")

    rospy.Service("check_effects_srv", CheckEffectsSrv, check_pddl_effects)
    rospy.Service("novel_effect_srv", NovelEffectsSrv, novel_effect)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
