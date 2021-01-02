#!/usr/bin/env python

import sys
import os 
import rospy
from threading import Thread

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

def solve_plan(solution, domainFilepath, problemFilepath):
    planner = Planner()
    solution['solution'] = planner.solve(domainFilepath, problemFilepath)
    return

def generate_plan(req):

    if req.problem.goals == []:
        print('Please specify at least one goal')
        return PlanGeneratorSrvResponse(ActionExecutionInfoList([]))

    domainFile = req.filename + '_domain.pddl'
    problemFile = req.filename + '_problem.pddl'
    solutionFile = req.filename + '_problem.pddl.soln'
    action_exclusions = req.action_exclusions

    dataFilepath = os.path.dirname(os.path.realpath(__file__)) + "/../data/"
    domainFilepath = dataFilepath + domainFile
    problemFilepath = dataFilepath + problemFile

    domain = KBDomainProxy(action_exclusions).domain

    # write to the files
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

    actionList = []  
    solution = {'solution' : None}

    try: 
        t = Thread(target=solve_plan, args=(solution, domainFilepath, problemFilepath,))
        t.start()
        t.join(20)
        success = t.is_alive() # Not currently used, 
                               # BUT: True if its still running after 5 seconds
    except TimeOutException as ex:
        print('No PDDL Solution Found: ' + str(ex))

    solution = solution['solution']

    if solution is not None:
        plan = getPlanFromPDDLactionList(solution)
        locBindings = KBActionLocsProxy().locBindings
        bindings = {}
        for bind in locBindings.bindings:
          bindings[bind.actionName] = bind.endEffectorInfo

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
            action_success = pddlActionExecutorProxy(actionName, args)
        except:
            failure_action = actionName
            return PlanExecutionOutcome(execution_success, goal_complete, failure_action)

        print(action_success)
        if action_success == 0:
            failure_action = actionName
            return PlanExecutionOutcome(execution_success, goal_complete, failure_action)

        effects = scenarioData().init
        effects_met = checkPddlEffects(actionName, args, preconditions, effects).effects_met

        if effects_met == False:
            failure_action = actionName
            # return PlanExecutionOutcome(execution_success, goal_complete, failure_action)

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

