#!/usr/bin/env python

import sys
import os 
import rospy

from std_msgs.msg import Empty


from util.file_io import * 
from util.data_conversion import getPlanFromPDDLactionList 
from util.pddl_parser.planner import Planner 
from agent.srv import * 
from pddl.msg import *
from pddl.srv import *
from environment.srv import * 


KBDomainProxy = rospy.ServiceProxy('get_KB_domain_srv', GetKBDomainSrv)
KBActionLocsProxy = rospy.ServiceProxy('get_KB_action_locs', GetKBActionLocsSrv)
pddlActionExecutorProxy = rospy.ServiceProxy('pddl_action_executor_srv', PddlExecutorSrv)
scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
checkPddlEffects = rospy.ServiceProxy('check_effects_srv', CheckEffectsSrv)

def generate_plan(req):
    domainFile = req.filename + '_domain.pddl'
    problemFile = req.filename + '_problem.pddl'
    solutionFile = req.filename + '_problem.pddl.soln'
    action_exclusions = req.action_exclusions

    dataFilepath = os.path.dirname(os.path.realpath(__file__)) + "/../data/"
    domainFilepath = dataFilepath + domainFile
    problemFilepath = dataFilepath + problemFile
    # solutionFilepath = dataFilepath + solutionFile

    domain = KBDomainProxy(action_exclusions).domain

    # print(domain)
    # # write to the files
    writeToDomainFile(domainFilepath, 
                      domain.name, 
                      domain.requirements,
                      domain.types, 
                      domain.predicates, 
                      domain.actions)

    writeToProblemFile(problemFilepath, 
                       req.problem.task,
                       req.problem.domain,
                       req.problem.objects,
                       req.problem.init, 
                       req.problem.goals)

    # pddlDriver = os.path.dirname(os.path.realpath(__file__)) + "/../../../Pyperplan/src/pyperplan.py" 
    # os.system('python3 ' + pddlDriver + ' ' + domainFilepath + ' ' + problemFilepath)

    # plan = getPlanFromSolutionFile(solutionFilepath)

    planner = Planner()
    solution = planner.solve(domainFilepath, problemFilepath)
    plan = getPlanFromPDDLactionList(solution)

    locBindings = KBActionLocsProxy().locBindings
    bindings = {}
    for bind in locBindings.bindings:
      bindings[bind.actionName] = bind.endEffectorInfo

    actionList = []    
    for act in plan:
        name = act['actionName']
        argVals = act['args']
        action = ActionExecutionInfo(name, argVals)
        actionList.append(action)

    return PlanGeneratorSrvResponse(ActionExecutionInfoList(actionList))

def execute_plan(req):
    execution_success = False
    goal_complete = None
    failure_action = None

    for action in req.actions.actions:
        actionName = action.actionName
        args = action.argVals 

        preconditions = scenarioData().init

        try:
            pddlActionExecutorProxy(actionName, args)
        except:
            failure_action = actionName
            return PlanExecutionOutcome(execution_success, goal_complete, failure_action)

        effects = scenarioData().init
        effects_met = checkPddlEffects(actionName, args, preconditions, effects).effects_met

        if effects_met == False:
            failure_action = actionName
            return PlanExecutionOutcome(execution_success, goal_complete, failure_action)

    execution_success = True
    return PlanExecutionOutcome(execution_success, goal_complete, failure_action)  

###########################################################################
def main():
    rospy.init_node("pddl_planner_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    rospy.Service("plan_generator_srv", PlanGeneratorSrv, generate_plan)
    rospy.Service("plan_executor_srv", PlanExecutorSrv, execute_plan)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
