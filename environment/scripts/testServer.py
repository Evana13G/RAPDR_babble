#!/usr/bin/env python

from environment.srv import *
import rospy

# from geometry_msgs.msg import (
#     PoseStamped,
#     Pose,
#     Point,
#     Quaternion,
# )
from std_msgs.msg import (
    Header,
    Empty,
)

# from action_primitive_variation.srv import APVSrv

def main():
    rospy.init_node("environment_test_node")
    rospy.wait_for_service('scenario_data_srv', timeout=60)
    try:
        b = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)
        # resp = b('obtain_object', 'left', 'block', None)
        resp = b()
        print("Response: ")
        print(resp.predicates)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()
