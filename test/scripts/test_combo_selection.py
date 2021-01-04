#!/usr/bin/env python

import rospy

from agent.srv import *
from environment.srv import *
from pddl.srv import *
from pddl.msg import *

from util.data_conversion import * 
from util.goal_management import * 

#### Service Proxies
scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
KBDomainProxy = rospy.ServiceProxy('get_KB_domain_srv', GetKBDomainSrv)
planGenerator = rospy.ServiceProxy('plan_generator_srv', PlanGeneratorSrv)
planExecutor = rospy.ServiceProxy('plan_executor_srv', PlanExecutorSrv)

def test(req):
    print("--------------------------------------")
    print("----- TESTING COMBO SELECTION -----")

    try:
        task_name = 'test'
        filename = 'test_1'
        goal = ['(cooking cup)']
        initStateInfo = scenarioData()
        initObjsIncludingLoc = extendInitLocs(initStateInfo, [])
        initObjsIncludingLoc['gripper'] = ['left_gripper']
        initObjsIncludingLoc['obj'] = ['cup', 'cover']
        objs = pddlObjectsStringFormat_fromDict(initObjsIncludingLoc)
        init = initStateInfo.init
        init = [x for x in init if 'right_gripper' not in x]
        init = [x for x in init if 'table' not in x]
        kb = KBDomainProxy(['test'])
        problem = Problem(task_name, kb.domain.name, objs, init, goal)
        plan = planGenerator(problem, filename, [])
        plan = plan.plan.actions
        print(plan)
        combos = generateAllCombos_dev(5, plan)
        for c in combos:
            print(c)
        return True 
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return False  

def main():
    rospy.init_node("test_combo_selection")
    rospy.Service("test_combo_selection_srv", EmptyTestSrv, test)
    rospy.spin()
    return 0 

if __name__ == "__main__":
    main()


