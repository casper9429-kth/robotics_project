import rospy
from behavior_tree.behavior_tree import BehaviorTree, Sequence, Selector, Inverter, SUCCESS, FAILURE, RUNNING
No = Not = Inverter

from rospy import Subscriber, Publisher, ServiceProxy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from path_planner.srv import Bool as BoolSrv, BoolSetter

class BrainNode:
    def __init__(self):
        rospy.init_node('brain_node')
        rospy.loginfo('Brain node started')
        self.behavior_tree = self._create_behavior_tree()
    
    def _create_behavior_tree(self):
        root = Sequence([Localize(), MoveToTarget()])
        behavior_tree = BehaviorTree(root, context=self.Context())
        return behavior_tree
    
    class Context:
        def __init__(self):
            self.anchor_id = 500

    def run(self):
        rate = rospy.Rate(10)
        result = RUNNING
        while not rospy.is_shutdown() and result != SUCCESS:
            result = self.behavior_tree.run()
            rate.sleep()
            

class Localize:
    def __init__(self):
        super().__init__()
        self.slam_ready = False
        self.slam_ready_subscriber = Subscriber('/slam_ready', Bool, self._slam_ready_callback, queue_size=1)
    
    def _slam_ready_callback(self, slam_ready_msg):
        self.slam_ready = slam_ready_msg.data

    def run(self):
        rospy.loginfo('Localizing')
        return SUCCESS if self.slam_ready else FAILURE


# returns the same goal but with noise added to x and y
def noisy(goal):
    import random
    goal.pose.position.x += random.uniform(-0.01, 0.01)
    goal.pose.position.y += random.uniform(-0.01, 0.01)
    return goal

    
class MoveToTarget:
    def __init__(self):
        super().__init__()
        self.goal_subscriber = Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1) # reads from rviz
        self.goal_publisher = rospy.Publisher('/test/goal', PoseStamped, queue_size=1) # sends to path planner
        
        self.start_path_tracker = rospy.ServiceProxy('/path_tracker/start', Trigger)
        self.path_tracker_is_running = ServiceProxy('/path_tracker/is_running', BoolSrv)
        
        self.toggle_path_planner_uninflation = ServiceProxy('/path_planner/toggle_uninflation', BoolSetter)
        self.last_goal_update = rospy.Time(0)
        self.cooldown_duration = rospy.Duration(5) # seconds

        self.goal = None
        self.is_running = False
    
    def run(self):
        rospy.loginfo('Moving to Target')
        if self.goal is None:
            return FAILURE
        if not self.is_running:
            self.is_running = True
            self.toggle_path_planner_uninflation(True)
            self.goal_publisher.publish(self.goal) # this is what we replace with the object's calculated pose
            self.last_goal_update = rospy.Time.now()
            self.start_path_tracker()
        elif not self.path_tracker_is_running().value:
            self.toggle_path_planner_uninflation(False)
            self.is_running = False
            return SUCCESS
        else:
            if rospy.Time.now() - self.last_goal_update > self.cooldown_duration:
                self.goal_publisher.publish(noisy(self.goal))
                self.last_goal_update = rospy.Time.now()
        return RUNNING
    
    def goal_callback(self, goal_msg):
        self.goal = goal_msg
        
        
if __name__ == '__main__':
    node = BrainNode()
    node.run()
    