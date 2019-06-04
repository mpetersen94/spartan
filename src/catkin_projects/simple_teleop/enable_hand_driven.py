import sys
import os
import numpy as np
import math
import time

# ROS
import rospy
import std_srvs.srv

# spartan
from spartan.manipulation.schunk_driver import SchunkDriver
# spartan ROS
import robot_msgs.msg

def do_main():
    rospy.init_node('hand_driven_plan', anonymous=True)

    # init gripper
    handDriver = SchunkDriver()

    # Then kick off hand_driven controller
    sp = rospy.ServiceProxy('plan_runner/init_hand_driven',
        robot_msgs.srv.StartStreamingPlan)
    init = robot_msgs.srv.StartStreamingPlanRequest()
    # init.force_guard.append(make_force_guard_msg(20.))
    res = sp(init)
    print("Started hand driven controller")

    def cleanup():
        rospy.wait_for_service("plan_runner/stop_plan")
        sp = rospy.ServiceProxy('plan_runner/stop_plan', std_srvs.srv.Trigger)
        init = std_srvs.srv.TriggerRequest()
        print sp(init)
        print("Done cleaning up and stopping streaming plan")

    rospy.on_shutdown(cleanup)
    rospy.spin()

if __name__ == "__main__":
    do_main()
