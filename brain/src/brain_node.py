#!/usr/bin/env python3

import rospy
from rospy import Subscriber, ServiceProxy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from actionlib import SimpleActionClient
from tf2_ros import Buffer, TransformListener

from arm.msg import ArmAction, ArmGoal, ArmResult, ArmFeedback
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
        context = {'anchor_id': 500,
                   'target_type': 'animal',
                   'is_holding_object': False,
                   'objects_remaining': 1,
                   'box_found': False,
                   'animal_found': False,}
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
    def __init__(self):
        self.buffer = Buffer(cache_time=rospy.Duration(60.0))
        self.listener = TransformListener(self.buffer)

    def run(self, context):
        rospy.loginfo('IsExplored')
        try:
            self.buffer.lookup_transform('map', 'aruco/detected1', rospy.Time(0))
            context['box_found'] = True
        except:
            pass
        try:
            self.buffer.lookup_transform('map', 'object/detected/Binky_1', rospy.Time(0))
            context['animal_found'] = True
        except:
            pass
        return SUCCESS if context['box_found'] and context['animal_found'] else FAILURE
    

class Explore(Leaf):
    def __init__(self):
        self.explore = ServiceProxy('/explore', Trigger)

    def run(self, context):
        rospy.loginfo('Explore')
        self.explore()
        return RUNNING


class ObjectsRemaining(Leaf):
    def run(self, context):
        rospy.loginfo('ObjectsRemaining')
        return SUCCESS if context['objects_remaining'] > 0 else FAILURE
    

class IsHoldingObject(Leaf):
    def run(self, context):
        rospy.loginfo('IsHoldingObject')
        return SUCCESS if context['is_holding_object'] else FAILURE
    

class CanPickUp(Leaf): # TODO
    def __init__(self):
        pass

    def run(self, context):
        rospy.loginfo('CanPickUp')
        # TODO: Check if distance to goal pose is less than some threshold
        return FAILURE
    

class GoToPickUp(Leaf): # TODO
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
    

class PickUp(Leaf): # TODO
    def __init__(self):
        self.action_client = SimpleActionClient('arm_actions', ArmAction) # TODO move this check to before the behavior tree or mod BT to check if initialized
        self.action_client.wait_for_server()
        self.running = False

    def run(self, context):
        rospy.loginfo('PickUp')
        if not self.running:
            self.running = True
            goal = ArmGoal()
            goal.action = 'pick_up'
            goal.type = context['target_type']
            goal.x = -0.15
            goal.y = 0.0
            goal.z = -0.13
            goal.yaw = 0.0

            def done_cb(state, result, context):
                self.running = False
                context['is_holding_object'] = True

            self.action_client.send_goal(goal, done_cb=done_cb)
        return RUNNING
    

class CanDropOff(Leaf): # TODO
    def run(self, context):
        rospy.loginfo('CanDropOff')
        return SUCCESS
    

class GoToDropOff(Leaf): # TODO
    def run(self, context):
        rospy.loginfo('GoToDropOff')
        return RUNNING
    

class DropOff(Leaf): # TODO
    def run(self, context):
        rospy.loginfo('DropOff')
        # TODO
        context['objects_remaining'] -= 1
        return RUNNING
    

class ReturnToAnchor(Leaf): # TODO
    def __init__(self):
        self.move_base_simple_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    def run(self, context):
        rospy.loginfo('ReturnToAnchor')
        pick_up_pose = PoseStamped()
        pick_up_pose.header.frame_id = 'map'
        pick_up_pose.pose.position.x = 0.0
        pick_up_pose.pose.position.y = 0.0
        pick_up_pose.pose.position.z = 0.0
        pick_up_pose.pose.orientation.x = 0.0
        pick_up_pose.pose.orientation.y = 0.0
        pick_up_pose.pose.orientation.z = 0.0
        pick_up_pose.pose.orientation.w = 1.0
        self.move_base_simple_publisher.publish(pick_up_pose)
        return RUNNING
        

if __name__ == '__main__':
    brain_node = BrainNode()
    brain_node.run()
