#!/usr/bin/env python3

from math import atan2

import rospy
from rospy import Subscriber, ServiceProxy, Publisher
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from actionlib import SimpleActionClient
from tf2_ros import Buffer, TransformListener, LookupException
from tf.transformations import quaternion_from_euler # using original tf is deprecated, but oh well

from arm.msg import ArmAction, ArmGoal
from path_planner.srv import Bool as BoolSrv
from behavior_tree.behavior_tree import BehaviorTree, Selector, Sequence, Inverter, Leaf, SUCCESS, FAILURE, RUNNING
from detection.msg import ObjectInstanceArray

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
        behavior_tree = BehaviorTree(root, context=self.Context())
        return behavior_tree
    
    class Context:
        def __init__(self):
            self.anchor_id = 500
            self.is_holding_object = False
            self.can_pick_up = False
            self.can_drop_off = False
            self.target = None
            self.detected_boxes = []

    def run(self):
        rate = rospy.Rate(10)
        result = RUNNING
        while not rospy.is_shutdown() and result != SUCCESS:
            result = self.behavior_tree.run()
            rate.sleep()
            
            
class Init(Leaf):
    def run(self):
        rospy.loginfo('Init')
        # TODO: wait for camera to warm up
        # TODO: test remove_old_instances for vision memory
        # TODO: wiggle to help see anchor
        # TODO: raise arm
        return SUCCESS


class IsLocalized(Leaf):
    def __init__(self):
        super().__init__()
        self.slam_ready = False
        self.slam_ready_subscriber = Subscriber('/slam_ready', Bool, self._slam_ready_callback, queue_size=1)
    
    def _slam_ready_callback(self, slam_ready_msg):
        self.slam_ready = slam_ready_msg.data

    def run(self):
        rospy.loginfo('IsLocalized')
        return SUCCESS if self.slam_ready else FAILURE


class Localize(Leaf):
    def run(self):
        rospy.loginfo('Localize')
        return RUNNING
    

class IsExplored(Leaf):
    def __init__(self):
        super().__init__()
        self.has_had_target = False
        self.stop_explore = ServiceProxy("explorer/stop", Trigger)

    def run(self):
        rospy.loginfo(f'IsExplored')
        if self.context.target:
            self.has_had_target = True

        if self.has_had_target and len(self.context.detected_boxes) > 0:
            self.stop_explore()
            return SUCCESS 
        else: 
            return FAILURE


class Explore(Leaf):
    def __init__(self):
        super().__init__()
        self.start_explore = ServiceProxy("explorer/start", Trigger)
        
        self.start_path_tracker = ServiceProxy('/path_tracker/start', Trigger)
        self.path_tracker_is_running = ServiceProxy('/path_tracker/is_running', BoolSrv)
        
        self.object_subscriber = Subscriber('/detection/object_instances', ObjectInstanceArray, self.object_instances_callback, queue_size=1)
        self.buffer = Buffer(cache_time=rospy.Duration(60.0))
        self.listener = TransformListener(self.buffer)
        
       

    def run(self):
        rospy.loginfo('Explore')
        if not self.path_tracker_is_running().value:
            self.start_path_tracker()

        self.start_explore() 
       
        for box_id in range(1,4):
            if self.buffer.can_transform('map', f'aruco/detected{box_id}', rospy.Time(0)) and box_id not in self.context.detected_boxes:
                self.context.detected_boxes.append(box_id)
                
        return RUNNING
    
    def object_instances_callback(self, msg):
        self.context.object_instances = msg.instances
        if len(msg.instances) > 0:
            if not self.context.target:
                self.context.target = msg.instances[-1]
            else:
                self.context.target = [instance for instance in msg.instances if instance.id == self.context.target.id][0] # TODO: this can break if we forget old object


class ObjectsRemaining(Leaf):
    def run(self):
        rospy.loginfo('ObjectsRemaining')
        objects_remaining = len(self.context.object_instances)
        return SUCCESS if objects_remaining > 0 else FAILURE
    

class IsHoldingObject(Leaf):
    def run(self):
        rospy.loginfo('IsHoldingObject')
        return SUCCESS if self.context.is_holding_object else FAILURE
    

class CanPickUp(Leaf):
    def run(self):
        rospy.loginfo('CanPickUp')
        return SUCCESS if self.context.can_pick_up else FAILURE


def calculate_pick_up_target_pose(object_position, tf_buffer):
    base_link = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'
    # get orientation from vector from base_link to target using quaternion_from_euler and math.atan2
    x = object_position.x - base_link.transform.translation.x
    y = object_position.y - base_link.transform.translation.y
    yaw = atan2(y, x)
    target_pose.pose.position.x = object_position.x
    target_pose.pose.position.y = object_position.y
    target_pose.pose.position.z = object_position.z
    q = quaternion_from_euler(0, 0, yaw)
    target_pose.pose.orientation.x = q[0]
    target_pose.pose.orientation.y = q[1]
    target_pose.pose.orientation.z = q[2]
    target_pose.pose.orientation.w = q[3]
    return target_pose


def distance_to_object(object_position, tf_buffer):
    base_link = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
    x = object_position.x - base_link.transform.translation.x
    y = object_position.y - base_link.transform.translation.y
    return (x**2 + y**2)**0.5


class GoToPickUp(Leaf):
    def __init__(self):
        super().__init__()
        self.move_base_simple_publisher = Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.start = ServiceProxy('/path_tracker/start', Trigger)
        self.path_tracker_is_running = ServiceProxy('/path_tracker/is_running', BoolSrv)
        self.is_running = False
        # listen to tf frame object/detected/instance_name to get target pose
        self.buffer = Buffer(cache_time=rospy.Duration(60.0))
        self.listener = TransformListener(self.buffer)
        self.update_distance_threshold = 0.15 # [m]

    def run(self):
        rospy.loginfo('GoToPickUp')
        if not self.context.target:
            return FAILURE
        try:
            if not self.is_running or distance_to_object(self.context.target.object_position, self.buffer) > self.update_distance_threshold:
                move_target_pose = calculate_pick_up_target_pose(self.context.target.object_position, self.buffer)
                self.move_base_simple_publisher.publish(move_target_pose)
        except LookupException:
            # TODO: we lost the target if we ever get here
            pass
        
        if not self.is_running:
            self.is_running = True
            self.start()
        elif not self.path_tracker_is_running().value:
            self.is_running = False
            self.context.can_pick_up = True
        return RUNNING
    

def category_name_to_type(category_name):
    if category_name in ["Red_cube", "Green_cube", "Blue_cube", "Wooden_cube"]:
        return 'cube'
    elif category_name in ["Red_ball", "Green_ball", "Blue_ball"]:
        return 'ball'
    elif category_name in ["Binky", "Hugo", "Slush", "Muddles", "Kiki", "Oakie"]:
        return 'animal'
    else:
        raise ValueError(f'Unknown category name {category_name}')


type_to_box_id = {
    'cube': 1,
    'ball': 2,
    'animal': 3
}


class PickUp(Leaf):
    def __init__(self):
        super().__init__()
        self.action_client = SimpleActionClient('arm_actions', ArmAction)
        self.arm_is_running = False
        self.forward_service = ServiceProxy('/move/forward/pick_up', Trigger)
        self.has_moved_forward = False
        self.reverse_service = ServiceProxy('/move/reverse/pick_up', Trigger)
        self.reverse_service_animal = ServiceProxy('/move/reverse/pick_up_animal', Trigger)
        self.has_reversed = False
        self.move_is_running = ServiceProxy('/move/is_running', BoolSrv)

    def run(self):
        rospy.loginfo('PickUp')
        if not self.has_moved_forward:
            self.forward_service()
            self.has_moved_forward = True
        elif not self.has_reversed and not self.move_is_running().value:
            object_type = category_name_to_type(self.context.target.category_name)
            if object_type == 'animal':
                self.reverse_service_animal()
            else:
                self.reverse_service()
            self.has_reversed = True
        elif not self.arm_is_running and not self.move_is_running().value:
            self.arm_is_running = True
            goal = ArmGoal()
            goal.action = 'pick_up'
            goal.type = category_name_to_type(self.context.target.category_name)
            # TODO: maybe get these from perception
            goal.x = -0.145
            goal.y = -0.04
            goal.z = -0.14 if goal.type == 'animal' else -0.13
            goal.yaw = 1.57 if goal.type == 'animal' else 0.0

            self.action_client.send_goal(goal, done_cb=self.done_callback)
        return RUNNING

    def done_callback(self, state, result):
        self.arm_is_running = False
        self.has_moved_forward = False
        self.has_reversed = False
        self.context.is_holding_object = True
        self.context.can_pick_up = False


class CanDropOff(Leaf):
    def run(self):
        rospy.loginfo('CanDropOff')
        return SUCCESS if self.context.can_drop_off else FAILURE


def calculate_drop_off_target_pose(target_tf_frame, tf_buffer):
    target = tf_buffer.lookup_transform('map', target_tf_frame, rospy.Time(0))
    base_link = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'
    target_pose.pose.position.x = target.transform.translation.x
    target_pose.pose.position.y = target.transform.translation.y
    target_pose.pose.position.z = target.transform.translation.z
    # get orientation from vector from base_link to target using quaternion_from_euler and math.atan2
    x = target.transform.translation.x - base_link.transform.translation.x
    y = target.transform.translation.y - base_link.transform.translation.y
    yaw = atan2(y, x)
    q = quaternion_from_euler(0, 0, yaw)
    target_pose.pose.orientation.x = q[0]
    target_pose.pose.orientation.y = q[1]
    target_pose.pose.orientation.z = q[2]
    target_pose.pose.orientation.w = q[3]
    return target_pose
    

# TODO: box orientation is not accounted for, pathplanning problem.
class GoToDropOff(Leaf):
    def __init__(self):
        super().__init__()
        self.move_base_simple_publisher = Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.start = ServiceProxy('/path_tracker/start', Trigger)
        self.path_tracker_is_running = ServiceProxy('/path_tracker/is_running', BoolSrv)
        self.is_running = False
        self.buffer = Buffer(cache_time=rospy.Duration(60.0))
        self.listener = TransformListener(self.buffer)

    def run(self):
        rospy.loginfo('GoToDropOff')
        try:
            box_frame = f'aruco/detected{self.context.detected_boxes[0]}'
            object_type = category_name_to_type(self.context.target.category_name)
            box_id = type_to_box_id[object_type]
            if box_id in self.context.detected_boxes:
                box_frame = f'aruco/detected{box_id}'
            move_target_pose = calculate_drop_off_target_pose(box_frame, self.buffer)

            self.move_base_simple_publisher.publish(move_target_pose)
        except LookupException:
            # TODO: we forgot the box if we ever get here
            pass

        if not self.is_running:
            self.is_running = True
            self.start()
        elif not self.path_tracker_is_running().value:
            self.is_running = False
            self.context.can_drop_off = True
                
        return RUNNING
    

class DropOff(Leaf):
    def __init__(self):
        super().__init__()
        self.delete_instance_publisher = Publisher('/detection/remove_instance', String, queue_size=1)
        self.action_client = SimpleActionClient('arm_actions', ArmAction)
        self.is_running = False
        self.speak_publisher = Publisher('/speaker/speech', String, queue_size=1) 
        self.reverse_service = ServiceProxy('/move/reverse/box', Trigger)
        self.reverse_is_running = ServiceProxy('/move/is_running', BoolSrv)
        self.has_reversed = False

    def run(self):
        rospy.loginfo('DropOff')
        if not self.is_running:
            self.is_running = True
            goal = ArmGoal()
            goal.action = 'drop_off'
            goal.type = category_name_to_type(self.context.target.category_name)
            # TODO: maybe get these from perception
            goal.x = -0.145
            goal.y = 0.0
            goal.z = -0.13
            goal.yaw = 0.0

            self.action_client.send_goal(goal, done_cb=self.arm_done_callback)
        elif not self.reverse_is_running().value and self.has_reversed:
            self.done()
        return RUNNING

    def arm_done_callback(self, state, result):
        self.reverse_service()
        self.has_reversed = True
        
    def done(self):
        self.delete_instance_publisher.publish(String(self.context.target.instance_name))
        self.is_running = False
        self.context.is_holding_object = False
        self.context.can_drop_off = False
        self.has_reversed = False
        if len(self.context.object_instances) > 0:
            self.context.target = self.context.object_instances[-1]
        else:
            self.context.target = None
        self.speak_publisher.publish("Object is in the box")
    

class ReturnToAnchor(Leaf):
    def __init__(self):
        super().__init__()
        self.move_base_simple_publisher = Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.start = ServiceProxy('/path_tracker/start', Trigger)
        self.path_tracker_is_running = ServiceProxy('/path_tracker/is_running', BoolSrv)
        self.speak_publisher = Publisher('/speaker/speech', String, queue_size=1) 
        self.is_running = False
        self.is_finished = False

    def run(self):
        rospy.loginfo('ReturnToAnchor')
        if self.is_finished:
            return SUCCESS
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
            self.context.can_drop_off = True
            self.is_finished = True
            self.speak_publisher.publish("I am done! Sleepy is going to sleep now.")
            return SUCCESS
                
        return RUNNING
        

if __name__ == '__main__':
    brain_node = BrainNode()
    brain_node.run()
