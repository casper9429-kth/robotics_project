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

from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import TransformStamped
import tf
import tf2_ros
import numpy as np


from arm.msg import ArmAction, ArmGoal
from arm.srv import ArmTrigger
from path_planner.srv import Bool as BoolSrv, BoolSetter
from behavior_tree.behavior_tree import BehaviorTree, Selector, Sequence, Inverter, Leaf, SUCCESS, FAILURE, RUNNING
from detection.msg import ObjectInstanceArray
from mapping.srv import CheckPolygon

No = Not = Inverter


class BrainNode:
    def __init__(self):
        rospy.init_node('brain_node')
        rospy.loginfo('Brain node started')
        self.behavior_tree = self._create_behavior_tree()
    
    def _create_behavior_tree(self):
        init = Init()
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
        root = Sequence([init, localize, explore, pick_up, drop_off, return_to_anchor])
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
            self.object_instances = []
            self.debug_messages = []

    def run(self):
        rate = rospy.Rate(10)
        result = RUNNING
        while not rospy.is_shutdown() and result != SUCCESS:
            self.behavior_tree.context.debug_messages = []
            result = self.behavior_tree.run()
            print(*self.behavior_tree.context.debug_messages, sep=' -> ')
            rate.sleep()
            
            
class Init(Leaf):
    def __init__(self):
        super().__init__()
        self.init_time = rospy.Time.now()
        self.straight = ServiceProxy('arm/steps/straight', ArmTrigger)
        
        self.arm_straight = False
        self.has_wiggled_1 = False
        self.has_wiggled_2 = False
        
        self.wiggle_service_1 = ServiceProxy('move/wiggle_1', Trigger)
        self.wiggle_service_2 = ServiceProxy('move/wiggle_2', Trigger)
        self.move_is_running = ServiceProxy('/move/is_running', BoolSrv)
        self.speak_publisher = Publisher('/speaker/speech', String, queue_size=1) 
        
    def run(self):
        self.context.debug_messages.append(type(self).__name__)
        if rospy.Time.now() - self.init_time > rospy.Duration(10.0): 
            if rospy.Time.now() - self.init_time > rospy.Duration(12.0) and self.arm_straight and self.has_wiggled_1 and self.has_wiggled_2:
                
                return SUCCESS
            else:
                if not self.arm_straight:
                    self.speak_publisher.publish("Good morning! I am ready to go!")
                    self.arm_straight = True
                    self.straight()
                elif self.arm_straight and not self.has_wiggled_1 and not self.move_is_running().value:
                    self.wiggle_service_1()
                    self.has_wiggled_1 = True
                elif self.has_wiggled_1 and not self.has_wiggled_2 and not self.move_is_running().value:
                    self.wiggle_service_2()
                    self.has_wiggled_2 = True
                
        return RUNNING


class IsLocalized(Leaf):
    def __init__(self):
        super().__init__()
        self.slam_ready = False
        self.slam_ready_subscriber = Subscriber('/slam_ready', Bool, self._slam_ready_callback, queue_size=1)
    
    def _slam_ready_callback(self, slam_ready_msg):
        self.slam_ready = slam_ready_msg.data

    def run(self):
        self.context.debug_messages.append(type(self).__name__)
        return SUCCESS if self.slam_ready else FAILURE


class Localize(Leaf):
    def run(self):
        self.context.debug_messages.append(type(self).__name__)
        return RUNNING
    

class IsExplored(Leaf):
    def __init__(self):
        super().__init__()
        self.stop_explore = ServiceProxy("explorer/stop", Trigger)
        self.is_explored = ServiceProxy("explorer/is_explored", Trigger)

    def run(self):
        self.context.debug_messages.append(type(self).__name__)
        
        if self.context.target is not None and len(self.context.detected_boxes) > 0:
            self.stop_explore()
            return SUCCESS 
        elif self.is_explored().success:
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
        self.context.debug_messages.append(type(self).__name__)
        if not self.path_tracker_is_running().value:
            self.start_path_tracker()

        self.start_explore()
        for box_id in range(1,4):
            if self.buffer.can_transform('map', f'aruco/detected{box_id}', rospy.Time(0)) and box_id not in self.context.detected_boxes:
                self.context.detected_boxes.append(box_id) # TODO: This should probably not be hidden in Explore
                
        return RUNNING
    
    def object_instances_callback(self, msg):
        self.context.object_instances = msg.instances
        if len(msg.instances) > 0:
            if not self.context.target or len([instance for instance in msg.instances if instance.id == self.context.target.id]) == 0:
                self.context.target = msg.instances[-1]
            else:
                self.context.target = [instance for instance in msg.instances if instance.id == self.context.target.id][0] 
                

class ObjectsRemaining(Leaf):
    def run(self):
        self.context.debug_messages.append(type(self).__name__)
        objects_remaining = len(self.context.object_instances)
        return SUCCESS if objects_remaining > 0 else FAILURE
    

class IsHoldingObject(Leaf):
    def run(self):
        self.context.debug_messages.append(type(self).__name__)
        return SUCCESS if self.context.is_holding_object else FAILURE
    

class CanPickUp(Leaf):
    def run(self):
        self.context.debug_messages.append(type(self).__name__)
        return SUCCESS if self.context.can_pick_up else FAILURE


def calculate_pick_up_target_pose(object_position, tf_buffer):
    base_link = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'
    # get orientation from vector from base_link to target usexplorer/starting quaternion_from_euler and math.atan2
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
        self.is_inside_workspace = ServiceProxy("/workspace/is_inside", CheckPolygon)
        self.delete_instance_publisher = Publisher('/detection/remove_instance', String, queue_size=1)
        self.toggle_path_planner_uninflation = ServiceProxy('/path_planner/toggle_uninflation', BoolSetter)
        self.is_running = False
        # listen to tf frame object/detected/instance_name to get target pose
        self.buffer = Buffer(cache_time=rospy.Duration(60.0))
        self.listener = TransformListener(self.buffer)
        self.update_distance_threshold = 0.1 # [m] distance before second goal is sent
        self.update_distance = None

    def run(self):
        self.context.debug_messages.append(type(self).__name__)
        if not self.context.target:
            return FAILURE
        
        #check if object is in the workspace, if not remove it from the list
        move_target_pose = calculate_pick_up_target_pose(self.context.target.object_position, self.buffer)
        x = move_target_pose.pose.position.x
        y = move_target_pose.pose.position.y
        if not self.is_inside_workspace(x, y).success:
            rospy.loginfo("Object is not in the workspace, removing it from the list")
            self.delete_instance_publisher.publish(String(self.context.target.instance_name))
            self.context.target = None
            return FAILURE
        
        distance_to_target = None
        try:
            distance_to_target = distance_to_object(self.context.target.object_position, self.buffer)
            if self.update_distance is None:
                self.update_distance = distance_to_target
            if not self.is_running:
                self.move_base_simple_publisher.publish(move_target_pose)
            elif self.update_distance - distance_to_target > self.update_distance_threshold:
                self.update_distance = distance_to_target
                self.move_base_simple_publisher.publish(move_target_pose)
        except LookupException:
            # TODO: we lost the target if we ever get here. We should reset things but this has never happened so whatever
            return FAILURE
        
        if not self.is_running:
            self.toggle_path_planner_uninflation(True) # true = uninflate so we can get close to the object
            self.is_running = True
            self.start()
        elif not self.path_tracker_is_running().value:
            self.toggle_path_planner_uninflation(False)
            self.is_running = False
            self.context.can_pick_up = True
            self.update_distance = None
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
        self.context.debug_messages.append(type(self).__name__)
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
        self.context.debug_messages.append(type(self).__name__)
        return SUCCESS if self.context.can_drop_off else FAILURE


# def calculate_drop_off_target_pose(target_tf_frame, tf_buffer):
#     target = tf_buffer.lookup_transform('map', target_tf_frame, rospy.Time(0))
#     base_link = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
#     target_pose = PoseStamped()
#     target_pose.header.frame_id = 'map'
#     target_pose.pose.position.x = target.transform.translation.x
#     target_pose.pose.position.y = target.transform.translation.y
#     target_pose.pose.position.z = target.transform.translation.z
#     # get orientation from vector from base_link to target using quaternion_from_euler and math.atan2
#     x = target.transform.translation.x - base_link.transform.translation.x
#     y = target.transform.translation.y - base_link.transform.translation.y
#     yaw = atan2(y, x)
#     q = quaternion_from_euler(0, 0, yaw)
#     target_pose.pose.orientation.x = q[0]
#     target_pose.pose.orientation.y = q[1]
#     target_pose.pose.orientation.z = q[2]
#     target_pose.pose.orientation.w = q[3]
#     return target_pose


# Old but gold implementation
# # TODO: box orientation is not accounted for, pathplanning problem.
# class GoToDropOff(Leaf):
#     def __init__(self):
#         super().__init__()
#         self.move_base_simple_publisher = Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
#         self.start = ServiceProxy('/path_tracker/start', Trigger)
#         self.path_tracker_is_running = ServiceProxy('/path_tracker/is_running', BoolSrv)
#         self.toggle_path_planner_uninflation = ServiceProxy('/path_planner/toggle_uninflation', BoolSetter)
#         self.is_running = False
#         self.buffer = Buffer(cache_time=rospy.Duration(60.0))
#         self.listener = TransformListener(self.buffer)

#     def run(self):
#         rospy.loginfo('GoToDropOff')
#         try:
#             box_frame = f'aruco/detected{self.context.detected_boxes[0]}'
#             object_type = category_name_to_type(self.context.target.category_name)
#             box_id = type_to_box_id[object_type]
#             if box_id in self.context.detected_boxes:
#                 box_frame = f'aruco/detected{box_id}'
#             move_target_pose = calculate_drop_off_target_pose(box_frame, self.buffer)

#             self.move_base_simple_publisher.publish(move_target_pose)
#         except LookupException:
#             # TODO: we forgot the box if we ever get here
#             pass

#         if not self.is_running:
#             self.toggle_path_planner_uninflation(True) # true = uninflate so we can get close to the box
#             self.is_running = True
#             self.start()
#         elif not self.path_tracker_is_running().value:
#             self.toggle_path_planner_uninflation(False)
#             self.is_running = False
#             self.context.can_drop_off = True
                
#         return RUNNING

class GoToDropOff(Leaf):
    """
    GoToDropOff is a leaf that makes the robot go to the drop off location. 
    It uses the path tracker to go to the drop off location.
    It works by:
    1. Getting the drop off location from the context
    2.1 Calulating the drop off location with a safe distance from the box
    2.2 Calulating the drop off location 
    3. Using a internal state to determine which drop off location to use (safe or actual), based on the distance to the safe drop off location
    4. If the aruco marker has changed, it will reset the internal state to safe_box and recalculate the drop off location
    5. Publishing the safe/actual -drop off location to the path tracker
    6. Starting the path tracker if it is not running
    """
    def __init__(self):
        super().__init__()
        self.move_base_simple_publisher = Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.start = ServiceProxy('/path_tracker/start', Trigger)
        self.path_tracker_is_running = ServiceProxy('/path_tracker/is_running', BoolSrv)
        self.toggle_path_planner_uninflation = ServiceProxy('/path_planner/toggle_uninflation', BoolSetter)
        self.is_running = False
        self.buffer = Buffer(rospy.Duration(60))
        self.listener = TransformListener(self.buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.state = 'safe_box' # safe_box, actual_box
        self.previous_box_frame = None
        self.threshold_distance_to_safe_box = 0.15 # [m], distance to the safe box pose before we use the actual box pose

    def run(self):
        self.context.debug_messages.append(type(self).__name__)
        try:
            box_frame = f'aruco/detected{self.context.detected_boxes[0]}'
            object_type = category_name_to_type(self.context.target.category_name)
            box_id = type_to_box_id[object_type]
            if box_id in self.context.detected_boxes:
                box_frame = f'aruco/detected{box_id}'
            move_target_pose_actual, move_target_pose_safe = self.calculate_drop_off_target_pose_with_safe_distance(box_frame, self.buffer)

            # Check if the box_frame has changed, if so, reset the state machine to safe_box state
            if self.previous_box_frame != box_frame:
                self.previous_box_frame = box_frame
                self.state = 'safe_box'
                # TODO update the drop off location for planner

            if move_target_pose_actual and move_target_pose_safe:
                # State machine: if robot is close to the safe box, switch to actual box to get closer
                
                if self.state == 'safe_box':
                    # check if we are close to the safe box
                    base_link_map_fram = self.buffer.lookup_transform('map', 'base_link', rospy.Time(0))
                    dx = base_link_map_fram.transform.translation.x - move_target_pose_safe.pose.position.x
                    dy = base_link_map_fram.transform.translation.y - move_target_pose_safe.pose.position.y
                    distance = np.sqrt(dx**2 + dy**2)
                    if distance > self.threshold_distance_to_safe_box:
                        self.move_base_simple_publisher.publish(move_target_pose_safe)
                    else:
                        self.state = 'actual_box'

                elif self.state == 'actual_box':
                    # check if we are close to the safe box
                    base_link_map_fram = self.buffer.lookup_transform('map', 'base_link', rospy.Time(0))
                    dx = base_link_map_fram.transform.translation.x - move_target_pose_actual.pose.position.x
                    dy = base_link_map_fram.transform.translation.y - move_target_pose_actual.pose.position.y
                    distance = np.sqrt(dx**2 + dy**2)
                    if distance > self.threshold_distance_to_safe_box:
                        self.move_base_simple_publisher.publish(move_target_pose_actual)
                    else:
                        # Return success, but wait because maybe there is more than my eye can see # TODO: fix this
                        pass
                    
            else:
                # No target found, print error message
                rospy.logwarn('Brain_node - GoToDropOff: No target found')

        except LookupException:
            # TODO: we forgot the box if we ever get here
            rospy.logwarn('Brain_node - GoToDropOff: LookupException')
            pass
        
        if not self.is_running:
            self.toggle_path_planner_uninflation(True) # true = uninflate so we can get close to the box
            self.is_running = True
            self.start()
        elif not self.path_tracker_is_running().value:
            self.toggle_path_planner_uninflation(False)
            self.is_running = False
            self.context.can_drop_off = True
            self.state = 'safe_box'
            
        return RUNNING
    
    def calculate_drop_off_target_pose_with_safe_distance(self,target_tf_frame, tf_buffer):
        
        target_tf_frame_safe = target_tf_frame + '_safe'
      
        try:
            target_actual = self.buffer.lookup_transform("map", target_tf_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("brain_node - GoToDropOf - calculate_drop_off_target_pose_with_safe_distance: Could not find actual target transform: %s", e)
            return None, None

        try:
            target_safe = self.buffer.lookup_transform("map", target_tf_frame_safe, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("brain_node - GoToDropOf - calculate_drop_off_target_pose_with_safe_distance: Could not find safe target transform: %s", e)
            return None, None

        # Lookup the actual target transform
        target_actual_return = None
        target_safe_return = None
        for target in [target_actual, target_safe]:
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
            if target == target_actual:
                target_actual_return = target_pose
            else:
                target_safe_return = target_pose


        return target_actual_return, target_safe_return
    

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
        self.context.debug_messages.append(type(self).__name__)
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

    # TODO: if we see new objects, we will not go back to the anchor, which
    # will cause problems next time we get to go_to_anchor since it will
    # already be running
    def run(self):
        self.context.debug_messages.append(type(self).__name__)
        if self.is_finished:
            return SUCCESS
        anchor_pose = PoseStamped()
        anchor_pose.header.frame_id = 'map'
        anchor_pose.pose.position.x = 0.0
        anchor_pose.pose.position.y = 0.0
        anchor_pose.pose.position.z = 0.0
        anchor_pose.pose.orientation.x = 0.0
        anchor_pose.pose.orientation.y = 0.0
        anchor_pose.pose.orientation.z = 0.0
        anchor_pose.pose.orientation.w = 1.0
        self.move_base_simple_publisher.publish(anchor_pose)
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
