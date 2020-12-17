#!/usr/bin/env python

import rospy

from agent.srv import *
from environment.srv import *
from pddl.srv import *
from pddl.msg import *

from util.data_conversion import * 

#### Service Proxies
scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
KBDomainProxy = rospy.ServiceProxy('get_KB_domain_srv', GetKBDomainSrv)
planGenerator = rospy.ServiceProxy('plan_generator_srv', PlanGeneratorSrv)
planExecutor = rospy.ServiceProxy('plan_executor_srv', PlanExecutorSrv)

def test(req):
    print("--------------------------------------")
    print("----- TESTING KB -----")

    try:
        task_name = 'test'
        filename = 'test_1'
        goal = ['(food_cooked )']
        initStateInfo = scenarioData()
        initObjsIncludingLoc = extendInitLocs(initStateInfo, [])
        initObjsIncludingLoc['gripper'] = ['left_gripper']
        initObjsIncludingLoc['burner'] = ['burner_1']
        objs = pddlObjectsStringFormat_fromDict(initObjsIncludingLoc)
        objs.append('burner_1 - burner')
        init = initStateInfo.init
        init = [x for x in init if 'right_gripper' not in x]
        problem = Problem(task_name, KBDomainProxy().domain.name, objs, init, goal)
        plan = planGenerator(problem, filename, [])
     
        # plan = plan.plan.actions
        plan = [a.actionName for a in plan.plan.actions]
        print("Plan: " + str(plan))
        return True 
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return False 

def main():
    rospy.init_node("test_kb")
    rospy.Service("test_kb_srv", EmptyTestSrv, test)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()


