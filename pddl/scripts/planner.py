#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np
import os 
import copy

import rospy
import rospkg
import rosbag

from std_msgs.msg import (
    Header,
    Empty,
)


from util.file_io import * 
from agent.srv import * 
from pddl.msg import *
from pddl.srv import *

KBDomainProxy = rospy.ServiceProxy('get_KB_domain_srv', GetKBDomainSrv)
KBActionLocsProxy = rospy.ServiceProxy('get_KB_action_locs', GetKBActionLocsSrv)
pddlActionExecutorProxy = rospy.ServiceProxy('pddl_action_executor_srv', PddlExecutorSrv)

def generate_plan(req):

    domainFile = req.filename + '_domain.pddl'
    problemFile = req.filename + '_problem.pddl'
    solutionFile = req.filename + '_problem.pddl.soln'

    dataFilepath = os.path.dirname(os.path.realpath(__file__)) + "/../data/"
    domainFilepath = dataFilepath + domainFile
    problemFilepath = dataFilepath + problemFile
    solutionFilepath = dataFilepath + solutionFile

    domain = KBDomainProxy().domain
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

    pddlDriver = os.path.dirname(os.path.realpath(__file__)) + "/../../../Pyperplan/src/pyperplan.py" 
    os.system('python3 ' + pddlDriver + ' ' + domainFilepath + ' ' + problemFilepath)

    plan = getPlanFromSolutionFile(solutionFilepath)

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
    try:
        for action in req.actions.actions:
            actionName = action.actionName
            args = action.argVals  
            pddlActionExecutorProxy(actionName, args)
        return PlanExecutorSrvResponse(1)        
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return PlanExecutorSrvResponse(0)


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
