#!/usr/bin/env python3

import rospy
from rospy import Subscriber
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import tf2_ros

from behavior_tree.behavior_tree import BehaviorTree, Selector, Sequence, Inverter, Leaf, SUCCESS, FAILURE, RUNNING

No = Not = Inverter


class BrainNode:
    def __init__(self):
        rospy.init_node('brain_node')
        rospy.loginfo('Brain node started')
        self.behavior_tree = self._create_behavior_tree()
    
    def _create_behavior_tree(self):
        localize = Selector([IsLocalized(), Localize()])
        explore = Selector([IsExplored(), Explore()])
        pick_up = Selector([No(ObjectsRemaining()),
                            IsHoldingObject(),
                            Sequence([Selector([CanPickUp(),
                                                GoToPickUp()]),
                                      PickUp()])])
        drop_off = Selector([Not(IsHoldingObject()),
                             Sequence([Selector([CanDropOff(),
                                                 GoToDropOff()]),
                                       DropOff()])])
        return_to_anchor = ReturnToAnchor()
        root = Sequence([localize, explore, pick_up, drop_off, return_to_anchor])
        behavior_tree = BehaviorTree(root, context=self._create_context())
        return behavior_tree
    
    def _create_context(self):
        context = {'anchor_id': 500,}
        return context 

    def run(self):
        self.behavior_tree.run_forever(rate=10)
        rospy.spin()


class IsLocalized(Leaf):
    def __init__(self):
        self.slam_ready = False
        self.slam_ready_subscriber = Subscriber('/slam_ready', Bool, self._slam_ready_callback, queue_size=1)
    
    def _slam_ready_callback(self, slam_ready_msg):
        self.slam_ready = slam_ready_msg.data

    def run(self, context):
        rospy.loginfo('IsLocalized')
        return SUCCESS if self.slam_ready else FAILURE
        
    

class Localize(Leaf):
    def run(self, context):
        rospy.loginfo('Localize')
        return RUNNING
    

class IsExplored(Leaf):
    def run(self, context):
        rospy.loginfo('IsExplored')
        return SUCCESS
    

class Explore(Leaf):
    def run(self, context):
        rospy.loginfo('Explore')
        return RUNNING


class ObjectsRemaining(Leaf):
    def run(self, context):
        rospy.loginfo('ObjectsRemaining')
        return SUCCESS
    

class IsHoldingObject(Leaf):
    def run(self, context):
        rospy.loginfo('IsHoldingObject')
        return FAILURE
    

class CanPickUp(Leaf):
    def run(self, context):
        rospy.loginfo('CanPickUp')
        return FAILURE
    

class GoToPickUp(Leaf):
    def __init__(self):
        self.move_base_simple_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    def run(self, context):
        rospy.loginfo('GoToPickUp')
        pick_up_pose = PoseStamped()
        pick_up_pose.header.frame_id = 'map'
        pick_up_pose.pose.position.x = 1.0
        pick_up_pose.pose.position.y = 0.0
        pick_up_pose.pose.position.z = 0.0
        pick_up_pose.pose.orientation.x = 0.0
        pick_up_pose.pose.orientation.y = 0.0
        pick_up_pose.pose.orientation.z = 0.0
        pick_up_pose.pose.orientation.w = 1.0
        self.move_base_simple_publisher.publish(pick_up_pose)
        return RUNNING
    

class PickUp(Leaf):
    def run(self, context):
        rospy.loginfo('PickUp')
        return RUNNING
    

class CanDropOff(Leaf):
    def run(self, context):
        rospy.loginfo('CanDropOff')
        return SUCCESS
    

class GoToDropOff(Leaf):
    def run(self, context):
        rospy.loginfo('GoToDropOff')
        return RUNNING
    

class DropOff(Leaf):
    def run(self, context):
        rospy.loginfo('DropOff')
        return RUNNING
    

class ReturnToAnchor(Leaf):
    def run(self, context):
        rospy.loginfo('ReturnToAnchor')
        return RUNNING
        

if __name__ == '__main__':
    brain_node = BrainNode()
    brain_node.run()