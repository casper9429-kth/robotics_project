#!/usr/bin/env python3

import rospy
from rospy import Subscriber, ServiceProxy, Publisher
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from actionlib import SimpleActionClient
from tf2_ros import Buffer, TransformListener

from arm.msg import ArmAction, ArmGoal, ArmResult, ArmFeedback
from path_planner.srv import Bool as BoolSrv
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
                   'animal_found': False,
                   'can_pick_up': False,
                   'can_drop_off': False,}
        return context 

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.behavior_tree.run()
            rate.sleep()


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
        rospy.loginfo(f'IsExplored - box: {context["box_found"]}, Binky: {context["animal_found"]}')
        try:
            self.buffer.lookup_transform('map', 'aruco/detected3', rospy.Time(0))
            context['box_found'] = True
        except:
            pass
        try:
            self.buffer.lookup_transform('map', 'object/detected/Binky1', rospy.Time(0))
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
    

class CanPickUp(Leaf):
    def run(self, context):
        rospy.loginfo('CanPickUp')
        return SUCCESS if context['can_pick_up'] else FAILURE
    

class GoToPickUp(Leaf):
    def __init__(self):
        self.move_base_simple_publisher = Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.start = ServiceProxy('/path_tracker/start', Trigger)
        self.path_tracker_is_running = ServiceProxy('/path_tracker/is_running', BoolSrv)
        self.is_running = False

    def run(self, context):
        rospy.loginfo('GoToPickUp')
        # TODO: get pick up pose from detection
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
        if not self.is_running:
            self.is_running = True
            self.start()
        elif not self.path_tracker_is_running().value:
            self.is_running = False
            context['can_pick_up'] = True
        return RUNNING
    

class PickUp(Leaf):
    def __init__(self):
        self.action_client = SimpleActionClient('arm_actions', ArmAction)
        self.is_running = False

    def run(self, context):
        rospy.loginfo('PickUp')
        if not self.is_running:
            self.is_running = True
            goal = ArmGoal()
            goal.action = 'pick_up'
            goal.type = context['target_type']
            goal.x = -0.145
            goal.y = 0.0
            goal.z = -0.13
            goal.yaw = 0.0

            def done_cb(state, result):
                self.is_running = False
                context['is_holding_object'] = True
                context['can_pick_up'] = False

            self.action_client.send_goal(goal, done_cb=done_cb)
        return RUNNING
    

class CanDropOff(Leaf):
    def run(self, context):
        rospy.loginfo('CanDropOff')
        return SUCCESS if context['can_drop_off'] else FAILURE
    

class GoToDropOff(Leaf):
    def __init__(self):
        self.move_base_simple_publisher = Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.start = ServiceProxy('/path_tracker/start', Trigger)
        self.path_tracker_is_running = ServiceProxy('/path_tracker/is_running', BoolSrv)
        self.is_running = False

    def run(self, context):
        rospy.loginfo('GoToDropOff')
        # TODO: get drop off pose from detection
        pick_up_pose = PoseStamped()
        pick_up_pose.header.frame_id = 'map'
        pick_up_pose.pose.position.x = 0.0
        pick_up_pose.pose.position.y = 1.0
        pick_up_pose.pose.position.z = 0.0
        pick_up_pose.pose.orientation.x = 0.0
        pick_up_pose.pose.orientation.y = 0.0
        pick_up_pose.pose.orientation.z = 0.0
        pick_up_pose.pose.orientation.w = 1.0
        self.move_base_simple_publisher.publish(pick_up_pose)
        if not self.is_running:
            self.is_running = True
            self.start()
        elif not self.path_tracker_is_running().value:
            self.is_running = False
            context['can_drop_off'] = True
                
        return RUNNING
    

class DropOff(Leaf):
    def __init__(self):
        self.action_client = SimpleActionClient('arm_actions', ArmAction)
        self.is_running = False

    def run(self, context):
        rospy.loginfo('DropOff')
        if not self.is_running:
            self.is_running = True
            goal = ArmGoal()
            goal.action = 'drop_off'
            goal.type = context['target_type']
            goal.x = -0.145
            goal.y = 0.0
            goal.z = -0.13
            goal.yaw = 0.0

            def done_cb(state, result):
                self.is_running = False
                context['is_holding_object'] = False
                context['can_drop_off'] = False
                context['objects_remaining'] -= 1

            self.action_client.send_goal(goal, done_cb=done_cb)
        return RUNNING
    

class ReturnToAnchor(Leaf):
    def __init__(self):
        self.move_base_simple_publisher = Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.start = ServiceProxy('/path_tracker/start', Trigger)
        self.path_tracker_is_running = ServiceProxy('/path_tracker/is_running', BoolSrv)
        self.is_running = False
        self.is_finished = False

    def run(self, context):
        if self.is_finished:
            return SUCCESS
        rospy.loginfo('ReturnToAnchor')
        # TODO: get drop off pose from detection
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
        if not self.is_running:
            self.is_running = True
            self.start()
        elif not self.path_tracker_is_running().value:
            self.is_running = False
            context['can_drop_off'] = True
            self.is_finished = True
            return SUCCESS # TODO check for this in BehaviorTree.run so we know when to stop
                
        return RUNNING
        

if __name__ == '__main__':
    brain_node = BrainNode()
    brain_node.run()
