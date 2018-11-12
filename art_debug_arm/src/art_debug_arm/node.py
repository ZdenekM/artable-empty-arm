#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import Bool
from actionlib import SimpleActionServer
from art_msgs.msg import ArmNavigationResult, ArmNavigationGoal, ArmNavigationAction, \
    PickPlaceGoal, PickPlaceAction, PickPlaceResult, PickPlaceFeedback, ObjInstance
import random
from copy import deepcopy


class FakeArmNavigation():

    def __init__(self, name):

        self._as = SimpleActionServer(name, ArmNavigationAction, self.action_cb)

    def action_cb(self, req):

        result = ArmNavigationResult()
        result.result = ArmNavigationResult.SUCCESS

        self._as.set_succeeded(result)


class FakeGrasping:
    ALWAYS = 0
    NEVER = 1
    RANDOM = 2

    def __init__(self):
        self.left_server = SimpleActionServer('arm_1/pp', PickPlaceAction,
                                              execute_cb=self.pick_place_left_cb)

        self.right_server = SimpleActionServer('arm_2/pp', PickPlaceAction,
                                               execute_cb=self.pick_place_right_cb)

        self.left_grasped_object_pub = rospy.Publisher("arm_1/grasped_object", ObjInstance, queue_size=1, latch=True)
        self.right_grasped_object_pub = rospy.Publisher("arm_2/grasped_object", ObjInstance, queue_size=1, latch=True)

        self.server = None
        self.objects = self.ALWAYS
        self.grasp = self.ALWAYS
        self.place = self.ALWAYS
        self.object_randomness = 0.8  # 80% of time object is known
        self.grasp_randomness = 0.4
        self.place_randomness = 0.4
        self.holding_left = False
        self.holding_right = False
        self.pick_length = 1  # how long (sec) takes to pick an object
        self.place_length = 1  # how long (sec) takes to place an object

        random.seed()

    def pick_place_left_cb(self, goal):
        self.pickplace_cb(goal, True)

    def pick_place_right_cb(self, goal):
        self.pickplace_cb(goal, False)

    def pickplace_cb(self, goal, left=True):
        result = PickPlaceResult()
        feedback = PickPlaceFeedback()
        if left:
            self.server = self.left_server
        else:
            self.server = self.right_server
        if not (goal.operation == PickPlaceGoal.PICK_OBJECT_ID or
                goal.operation == PickPlaceGoal.PICK_FROM_FEEDER or
                goal.operation == PickPlaceGoal.PLACE_TO_POSE):
            result.result = PickPlaceResult.BAD_REQUEST
            rospy.logerr("BAD_REQUEST, Unknown operation")
            self.server.set_aborted(result, "Unknown operation")

            return

        if self.objects == self.ALWAYS:
            pass
        elif self.objects == self.NEVER:
            result.result = PickPlaceResult.BAD_REQUEST
            rospy.logerr("BAD_REQUEST, Unknown object id")
            self.server.set_aborted(result, "Unknown object id")
            return
        elif self.objects == self.RANDOM:
            nmbr = random.random()
            if nmbr > self.object_randomness:
                result.result = PickPlaceResult.BAD_REQUEST
                rospy.logerr("BAD_REQUEST, Unknown object id")
                self.server.set_aborted(result, "Unknown object id")
                return

        grasped = False

        if goal.operation == PickPlaceGoal.PICK_OBJECT_ID or goal.operation == PickPlaceGoal.PICK_FROM_FEEDER:
            rospy.sleep(self.pick_length)
            if (left and self.holding_left) or (not left and self.holding_right):
                result.result = PickPlaceResult.FAILURE
                rospy.logerr("Failure, already holding object in " +
                             "left" if left else "right" + "arm")
                self.server.set_aborted(
                    result, "Already holding object in " + "left" if left else "right" + "arm")
                return
            if self.grasp == self.ALWAYS:
                grasped = True
                pass
            elif self.grasp == self.NEVER:
                result.result = PickPlaceResult.FAILURE
                rospy.logerr("FAILURE, Pick Failed")
                self.server.set_aborted(result, "Pick Failed")
                return

            tries = 5
            max_attempts = 5
            while tries > 0:
                feedback.attempt = (max_attempts - tries) + 1
                tries -= 1
                self.server.publish_feedback(feedback)

                if self.grasp == self.RANDOM:
                    nmbr = random.random()
                    if nmbr < self.grasp_randomness:
                        grasped = True
                if grasped:
                    break

            if self.server.is_preempt_requested():
                self.server.set_preempted(result, "Pick canceled")
                rospy.logerr("Preempted")
                return

            if not grasped:
                result.result = PickPlaceResult.FAILURE
                self.server.set_aborted(result, "Pick failed")
                rospy.logerr("FAILURE, Pick Failed")
                return
            else:
                o = ObjInstance()
                o.object_id = goal.object
                o.object_type = goal.object
                if left:
                    self.holding_left = True
                    self.left_grasped_object_pub.publish(o)
                else:
                    self.holding_right = True
                    self.right_grasped_object_pub.publish(o)
        placed = False
        if goal.operation == PickPlaceGoal.PLACE_TO_POSE:
            rospy.sleep(self.place_length)
            if (left and not self.holding_left) or (not left and not self.holding_right):
                result.result = PickPlaceResult.FAILURE
                rospy.logerr("Failure, already holding object in " +
                             "left" if left else "right" + "arm")
                self.server.set_aborted(
                    result, "Already holding object in " + "left" if left else "right" + "arm")
                return
            if self.place == self.ALWAYS:
                placed = True
                pass
            elif self.place == self.NEVER:
                result.result = PickPlaceResult.FAILURE
                self.server.set_aborted(result, "Place Failed")
                rospy.logerr("FAILURE, Place Failed")
                return

            tries = 5
            max_attempts = 5
            while tries > 0:
                feedback.attempt = (max_attempts - tries) + 1
                tries -= 1
                self.server.publish_feedback(feedback)

                if self.place == self.RANDOM:
                    nmbr = random.random()
                    if nmbr < self.place_randomness:
                        placed = True
                if placed:
                    break
            if not placed:
                result.result = PickPlaceResult.FAILURE
                self.server.set_aborted(result, "Place failed")
                rospy.logerr("FAILURE, Place Failed")
                return
            else:
                o = ObjInstance()
                if left:
                    self.holding_left = False
                    self.left_grasped_object_pub.publish(o)
                else:
                    self.holding_right = False
                    self.right_grasped_object_pub.publish(o)

        result.result = PickPlaceResult.SUCCESS
        self.server.set_succeeded(result)
        rospy.loginfo("SUCCESS")
        print("Finished")


class ArtDebugArm(object):

    def __init__(self):
        self.left_arm_mann = False
        self.right_arm_mann = False

        self.left_interaction_on = rospy.Service("arm_1/interaction/on",
                                                 Trigger, self.left_interaction_on_cb)
        self.left_interaction_off = rospy.Service("arm_1/interaction/off",
                                                  Trigger, self.left_interaction_off_cb)
        self.left_get_ready = rospy.Service("arm_1/get_ready", Trigger,
                                            self.left_interaction_get_ready_cb)
        self.left_move_to_user = rospy.Service("arm_1/move_to_user", Trigger,
                                               self.left_interaction_move_to_user_cb)
        self.left_int_pub = rospy.Publisher("arm_1/interaction/state", Bool, queue_size=1, latch=True)

        self.right_interaction_on = rospy.Service("arm_2/interaction/on",
                                                  Trigger, self.right_interaction_on_cb)
        self.right_interaction_off = rospy.Service("arm_2/interaction/off",
                                                   Trigger, self.right_interaction_off_cb)
        self.right_get_ready = rospy.Service("arm_2/get_ready", Trigger,
                                             self.right_interaction_get_ready_cb)
        self.right_move_to_user = rospy.Service("arm_2/move_to_user", Trigger,
                                                self.right_interaction_move_to_user_cb)
        self.right_int_pub = rospy.Publisher("arm_2/interaction/state", Bool, queue_size=1, latch=True)

        self.left_int_pub.publish(False)
        self.right_int_pub.publish(False)

        FakeArmNavigation("arm_1/manipulation")
        FakeArmNavigation("arm_2/manipulation")
        FakeGrasping()

    def get_ready_cb(self, req):
        resp = TriggerResponse()
        resp.success = True
        return resp

    def left_interaction_on_cb(self, req):
        resp = TriggerResponse()
        if self.left_arm_mann:
            rospy.logerr('arm_1 already in interactive mode')
            resp.success = False
        else:
            rospy.loginfo('arm_1 interactive mode ON')
            self.left_arm_mann = True
            resp.success = True
            self.left_int_pub.publish(True)
        return resp

    def left_interaction_off_cb(self, req):
        resp = TriggerResponse()
        if not self.left_arm_mann:
            rospy.logerr('arm_1 already in normal mode')
            resp.success = False
        else:
            rospy.loginfo('arm_1 interactive mode OFF')
            self.left_arm_mann = False
            resp.success = True
            self.left_int_pub.publish(False)
        return resp

    def left_interaction_get_ready_cb(self, req):

        if self.left_arm_mann:
            rospy.logerr('arm_1 in interactive mode')
        else:
            rospy.loginfo('arm_1 in GET READY position')
            pass
        resp = TriggerResponse()
        resp.success = True
        return resp

    def left_interaction_move_to_user_cb(self, req):

        if self.left_arm_mann:
            rospy.logerr('arm_1 in interactive mode')
        else:
            rospy.loginfo('arm_1 moved TO USER')
        resp = TriggerResponse()
        resp.success = True
        return resp

    def right_interaction_on_cb(self, req):
        resp = TriggerResponse()

        if self.right_arm_mann:
            rospy.logerr('arm_2 already in interactive mode')
            resp.success = False
        else:
            rospy.loginfo('arm_2 interactive mode ON')
            resp.success = True
            self.right_arm_mann = True
            self.left_int_pub.publish(True)

        return resp

    def right_interaction_off_cb(self, req):
        resp = TriggerResponse()

        if not self.right_arm_mann:
            rospy.logerr('arm_2 already in normal mode')
            resp.success = False
        else:
            rospy.loginfo('arm_2 interactive mode OFF')
            resp.success = True
            self.right_arm_mann = False
            self.left_int_pub.publish(False)

        return resp

    def right_interaction_get_ready_cb(self, req):

        if self.right_arm_mann:
            rospy.logerr('arm_2 in interactive mode')
        else:
            rospy.loginfo('arm_2 in GET READY position')
        resp = TriggerResponse()
        resp.success = True
        return resp

    def right_interaction_move_to_user_cb(self, req):

        if self.right_arm_mann:
            rospy.logerr('arm_2 in interactive mode')
        else:
            rospy.loginfo('arm_2 moved TO USER')
        resp = TriggerResponse()
        resp.success = True
        return resp


if __name__ == '__main__':
    rospy.init_node('art_debug_arm', log_level=rospy.INFO)

    try:
        node = ArtDebugArm()
        rospy.spin()

    except rospy.ROSInterruptException:

        node = None