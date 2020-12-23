#!/usr/bin/env python

import rospy
import copy

from environment.srv import * 
from pddl.srv import *
from pddl.msg import *
from agent.srv import *

from util.knowledge_base.knowledge_base import KnowledgeBase, StaticPredicate, Action, Variable

KB = KnowledgeBase()
getObjLoc = rospy.ServiceProxy('object_location_srv', ObjectLocationSrv)
executionInfo = rospy.ServiceProxy('get_offset', GetHardcodedOffsetSrv)
orientationSolver = rospy.ServiceProxy('calc_gripper_orientation_pose', CalcGripperOrientationPoseSrv)

def handle_domain_req(req):
    domainDict = KB.getDomainData(req.action_exclusions)
    domainName = domainDict['domain']
    types = domainDict['types']
    predicates = domainDict['predicates']
    requirements = domainDict['requirements']
    actions = domainDict['actions']
    return Domain(domainName, requirements, types, predicates, actions)

def handle_pddlLocs_req(req):
    domainDict = KB.getDomainData()
    return(domainDict['pddlLocs'])

def handle_action_locs_req(req):
    return KB.getActionsLocs()

def get_param_options(req):
    name = req.actionName
    param = req.paramName
    action = KB.getAction(name)
    return action.getParam(param).getPossibleVals()

def get_action_info(req):
    name = req.actionName
    action = KB.getAction(name)
    argNames = action.getExecutionArgNames()
    paramNames = [x.getName() for x in action.getParams()]
    paramDefaults = [x.getDefaultVal() for x in action.getParams()]
    paramMins = [x.getMin() for x in action.getParams()]
    paramMaxs = [x.getMax() for x in action.getParams()]
    discreteVals = [DiscreteParamVals(x.getPossibleVals()) for x in action.getParams()]

    return ActionInfo(name, 
                      argNames, 
                      paramNames, 
                      paramDefaults, 
                      paramMins, 
                      paramMaxs, 
                      discreteVals)

def handle_get_pddl_instatiations(req):    
    name = req.actionName
    action = KB.getAction(name)
    args = req.orderedArgs 
    print(args)
    # info = executionInfo('cover', 'left')
    # info2 = orientationSolver('left_gripper', 'cover', 'left')

    # This is not a correct assumption to make
    # locs = [poseStampedToString(getObjLoc(x).location) for x in args]
    passed_locs = [x for x in args if '.' in x]
    if passed_locs == []:
        locs = ['A']
    else:
        args = [x for x in args if '.' not in x]
        locs = passed_locs
    preConds = action.get_instatiated_preconditions(args, locs)
    effects = action.get_instatiated_effects(args, locs)

    return ActionPDDLBinding(name, preConds, effects)


def add_action_to_KB(req):  

    new_name = req.new_action_name
    args = req.args
    param_names = req.param_names
    param_assignments = req.param_assignments
    new_effects = req.new_effects
    
    assert(len(param_names) == len(param_assignments))
    new_action = copy.deepcopy(KB.getAction(req.orig_action_name))
    new_action.setName(new_name)
    pddl_args = [x.getName() for x in new_action.getArgs() if 'loc' not in x.getName()]
    assert(len(args) == len(pddl_args))

    for i in range(len(new_effects)):
        effect = new_effects[i]

        # if 'not' not in effect:
        #     operator = effect[1:].split()[0]
        #     instatiated_pred_args = effect.replace(operator, '')[:-1].split()
        #     static_pred_args, new_args = parse_and_map_predicate_args(instatiated_pred_args, args, pddl_args)
        #     pred = StaticPredicate(operator, static_pred_args)
        # else:
        #     operator = effect[6:].split()[0]
        #     instatiated_pred_args = effect.replace(('(not (' + operator), '')[:-2].split()
        #     static_pred_args, new_args = parse_and_map_predicate_args(instatiated_pred_args, args, pddl_args)
        #     pred = StaticPredicate('not', [StaticPredicate(operator, static_pred_args)])


        operator = effect[1:].split()[0] if 'not' not in effect else effect[6:].split()[0]
        instatiated_pred_args = effect.replace(operator, '')[:-1].split() if 'not' not in effect else  effect.replace(('(not (' + operator), '')[:-2].split()
        static_pred_args, new_args = parse_and_map_predicate_args(instatiated_pred_args, args, pddl_args)
        pred = StaticPredicate(operator, static_pred_args) if 'not' not in effect else StaticPredicate('not', [StaticPredicate(operator, static_pred_args)])

        new_action.addEffect(pred)
        for new_arg in new_args:
            new_action.addArg(Variable(new_arg, 'obj'))
            new_action.addExecutionArgName(new_arg.replace('?arg', 'object'))

    for i in range(len(param_names)):
        new_action.setParamDefault(param_names[i], param_assignments[i])

    KB.addAction(new_action)

    return AddActionToKBSrvResponse(True)

    # except:
    #     return AddActionToKBSrvResponse(False)

def parse_and_map_predicate_args(instatiated_pred_args, args, pddl_args):            
    static_pred_args =[]
    new_args = []
    for i in range(len(instatiated_pred_args)):
        arg = instatiated_pred_args[i]
        try:
            j = args.index(arg)
            static_pred_args.append(pddl_args[j])
        except:
            new_arg = '?arg' + str(i)
            new_args.append(new_arg)
            static_pred_args.append(new_arg)
    return static_pred_args, new_args

def get_param_options(req):
    action = KB.getAction(req.actionName)
    vals = action.getParam(req.paramName).getPossibleVals()
    return vals
################################################################################

def main():
    rospy.init_node("knowledge_base_node")

    rospy.Service("get_KB_domain_srv", GetKBDomainSrv, handle_domain_req)
    rospy.Service("get_KB_action_info_srv", GetKBActionInfoSrv, get_action_info)
    rospy.Service("get_KB_action_locs", GetKBActionLocsSrv, handle_action_locs_req)
    rospy.Service("get_KB_pddl_locs", GetKBPddlLocsSrv, handle_pddlLocs_req)
    rospy.Service("get_pddl_instatiations_srv", GetActionPDDLBindingSrv, handle_get_pddl_instatiations)
    rospy.Service("add_action_to_KB_srv", AddActionToKBSrv, add_action_to_KB)
    rospy.Service("get_param_options_srv", GetParamOptionsSrv, get_param_options)

    rospy.spin()

    return 0 
################################################################################

if __name__ == "__main__":
    main()


