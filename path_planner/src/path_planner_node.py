#!/usr/bin/env python3
import heapq
import rospy
from functools import total_ordering
import numpy as np
import math
#import move_base_msgs.msg as mb
from dataclasses import dataclass,field
import time

from typing import Dict,Tuple
from scipy.interpolate import CubicSpline
from queue import PriorityQueue
from std_msgs.msg import Bool
from path_planner.srv import BoolSetter, Bool as BoolSrv, BoolSetterResponse, BoolResponse as BoolSrvResponse
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import tf
import tf2_ros 
import tf2_geometry_msgs
from mapping.msg import GridMapMsg
#from mapping.grid_map.grid_map import GridMap


@total_ordering
@dataclass
class Node:
    x: float
    y: float
    goal: tuple
    g: float = field(init=False)
    h: float = field(init=False)
    f: float = field(init=False)
    parent: object = field(default=None)
    #children: list = field(default_factory=list, init=False)

    # Makes sure that the stuff to be initalized is. 
    # Needed since dataclasses cant handle initalizing functions
    def __post_init__(self):
        if self.parent == None:
            self.g = 0
        else:
            parent_pos =[pos for pos in self.parent.position()]
            self.g = self.parent.g + math.dist([self.x,self.y],parent_pos)
            #self.g = self.parent.g +self.manhattan(parent_pos,(self.x,self.y))
        #self.h = self.dist_to_goal(self.goal)
        self.h = self.manhattan(self.goal,(self.x,self.y))
        self.f = self.g + self.h

    def manhattan(self,a, b):
        D=2
        return D*sum(abs(val1-val2) for val1, val2 in zip(a,b))
    
    def __lt__(self,other):
        return self.f < other.f
    """def __eq__(self, other):
        return self.f == other.f"""

    def dist_to_goal(self, goal):
        return math.dist((self.x, self.y), goal)
    
    def set_g(self):
        self.g = self.parent.f
    
    def position(self):
        return (self.x,self.y)
    
    def as_array(self):
        return np.array([self.x, self.y, self.goal[0], self.goal[1], self.g, self.h, self.f])
    
    def __repr__(self) -> str:
        return f'x = {self.x}, y = {self.y}, goal = {self.goal}, g = {self.g}, h = {self.h}, f = {self.f}'


class A_star():

    def __init__(self):
        self.current_goal_index = None
        self.should_uninflate_around_goal = False

        #self.client = actionlib.SimpleActionClient('path_tracker', move_base_msgs.msg.MoveBaseAction)
        self.map_coords = None
        self.map_resolution = None
        self.bbminx = None
        self.bbminy = None
        self.bbmaxx = None
        self.bbmaxy = None
        self.t_stamp = None
        self.inflation_r_index = None
        self.origo_index_i = None
        self.origo_index_j = None

        self.iterations = 1000
        self.goal_reached = Bool()
        self.goal_reached.data = False
        
        # subscriber 
        self.sub_map = rospy.Subscriber("/map/GridMap", GridMapMsg, self.map_callback)
        #self.sub_goal_reached = rospy.Subscriber('/path_tracker/feedback', Bool, self.goal_reached_callback)
        self.goal_reached_pub = rospy.Subscriber('/goal_reached', Bool,self.goal_reached_callback)

        #self.sub_start_and_goal = rospy.Subscriber("/start_and_goal", PoseArray, self.start_and_goal_callback)
        
        self.complete_path = None
        
        
        # tf things
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # robot pose in base_link
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'base_link'
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 1.0
        self.t_stamp = None
        
    ####################################  CALLBACK  #######################

    def goal_reached_callback(self,msg): # TODO figure it out
        self.goal_reached = msg.data

    def map_callback(self,msg):
        self.map_resolution = msg.resolution
        self.bbminx = msg.bbminx
        self.bbminy = msg.bbminy
        self.bbmaxx = msg.bbmaxx
        self.bbmaxy = msg.bbmaxy
        self.t_stamp = msg.header.stamp
        self.inflation_r_index = msg.inflation_r_index
        self.origo_index_i = msg.origo_index_i
        self.origo_index_j = msg.origo_index_j
        #print('Path planner: minx = ', self.bbminx, 'miny = ', self.bbminy, 'maxx = ', self.bbmaxx, 'maxy = ', self.bbmaxy)
        self.map_coords = msg.data 

    
    #####################################  MAP STUFF ##########################################
    
    
    def get_index_of_pos(self,x,y):
        """Return index of position in map grid, if not given geofence, return None"""
        """if not self.given_geofence:
            rospy.logwarn("Path planner: No geofence given, but trying to get map grid")
            return None"""
        
        x_index = int((x-self.bbminx)/self.map_resolution)
        y_index = int((y-self.bbminy)/self.map_resolution)
        
        return x_index, y_index
    
    def get_value_of_index(self,i,j):
        """Return value of index in map grid, if not given geofence, return None"""
        """if not self.given_geofence:
            rospy.logwarn("Path planner: No geofence given, but trying to get map grid")
            return None"""
        
        # if out of bounds, return 1

        if i < 0 or i > int((self.bbmaxx-self.bbminx)/self.map_resolution) or j < 0 or j > int((self.bbmaxy-self.bbminy)/self.map_resolution):
            rospy.loginfo('Failed, now printing 1 ')
            return 1
        
        # if we turn off inflation around target, make sure we return the corrected value
        if self.should_uninflate_around_goal and self.current_goal_index:
            # +1 might not be needed
            if math.dist((i, j), self.current_goal_index) <= self.inflation_r_index + 1:
                rospy.loginfo('this should print for sure')
                return 0 # 0 is free space
        
        return self.map_coords[i].data[j]
    
    def get_pos_of_index(self,i,j):
        """Return position of index in map grid, if not given geofence, return None"""
        """if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None"""
        x = round(self.bbminx + i*self.map_resolution,4)
        y = round(self.bbminy + j*self.map_resolution,4)
        
        return x, y
    
    def get_value_of_pos(self,x,y):
        """Return value of position in map grid, if not given geofence, return None"""
        """if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None"""
        
        i,j = self.get_index_of_pos(x,y)
        
        return self.get_value_of_index(i,j)
        
    # Name TBD 
    def obstacle_on_calc_path(self,path_smooth: list,path_rough: list):
        # scan current path if it does not collide with object
        start = self.get_start_in_map()
        check = all(point in path_smooth for point in path_rough)
        if check:
            start_index = 0 
            # checks if the currently calculated path has a new obstacle
            for point in path_smooth:
                index = path_rough.index(point)

                for point in path_rough[start_index:index]:
                    if self.get_value_of_pos(point[0],point[1])> 0.8:
                        return True
                start_index = index
            return False
        # in case of error just return True for recalculation
        #rospy.logwarn('Path Planner: not all points match')    
        return True
            

        

    ################################## A STAR ###################################################
    
    def distance(self,node1: Node,node2:Node):
        return math.dist(node1.position(),node2.position())

    def reconstruct_path(self,node: Node):
        pathlist = []
        while node.parent is not None:  # found target
            pathlist.append(node)
            node = node.parent
        pathlist.append(node) 
        path = [(node.x,node.y) for node in pathlist]
        path.reverse()
        #rospy.loginfo(f'FOUND PATH')
        return path
    

    
    # checks if inbounds according to the paramethes of the map
    def is_in_bounds(self,node):
        if self.bbminy < node.y < self.bbmaxy:
            if self.bbminx < node.x < self.bbmaxx:
                return True
        else:
            #print('Path planner: node is out of bounds')
            return False
        #self.map.is_point_in_polygon(new_x,new_y,self.map.geofence_list):

    def generate_neighbours(self,node):
        neighbourlist = {}
        buffer = 0.1
        dx = self.map_resolution # 5 cm
        dy = self.map_resolution
        for longditude in range(-1,2,1):
            for latitude in range(-1,2,1):
                new_x = node.x + dx*longditude
                new_y = node.y + dy*latitude
                neighbour = Node(new_x, new_y, parent = node, goal= node.goal)
                if self.is_in_bounds(neighbour):
                    #rospy.loginfo(f'Path planner: neighbour is in bounds {neighbour.position()}')
                    #TODO: check if this fucks up the explorer
                    if self.get_value_of_pos(new_x,new_y)>=(0.8): # will always work due to checking inbounds
                        neighbour.g = np.inf
                        neighbour.f = neighbour.g + neighbour.h
                    
                    neighbourlist[neighbour.position()] = neighbour

        return neighbourlist
    

    def get_robot_pose_in_map(self):
        try:                                   
            transform_base_link_2_map = self.tfBuffer.lookup_transform('map','base_link', self.t_stamp,rospy.Duration(0.5)) # lookup_transform('target frame','source frame', time.stamp, rospy.Duration(0.5))
            self.robot_pose = tf2_geometry_msgs.do_transform_pose(self.pose, transform_base_link_2_map)
            return self.robot_pose
        except:
            print('Path planner: No transform found')
            return None
        
        
    def get_start_in_map(self):
        robot_pose = self.get_robot_pose_in_map()
        i,j = self.get_index_of_pos(robot_pose.pose.position.x,robot_pose.pose.position.y)
        x,y = self.get_pos_of_index(i,j)
        
        return (x,y)
    

        
    ############################## Main Function #################################
    
    def path(self,goal,start = None):
        rospy.loginfo("A* path planning started")   
        start = self.get_robot_pose_in_map()
        # For debugging purposes
        #rospy.loginfo(f'Path:planner: goal {goal.pose.position.x,goal.pose.position.y}')
        if start == None:
            return None,[]
        else: 
            start = (start.pose.position.x,start.pose.position.y)
        goal = (goal.pose.position.x,goal.pose.position.y)
        self.current_goal_index = self.get_index_of_pos(*goal) # ugly but works, used in get_value_of_index to uninflate around goal

        #start = (start.pose.position.x,start.pose.position.y)
        if start == None: # if no start is given use regular
            start = self.get_start_in_map()
        else:
            start = start # This is for recalculation of path from first node
        #print(f'Path planner: start {start}')
        heap = []
        heapq.heapify(heap)
        openset = {}
        closedset = {}
        #heapify(openset)
        #best_path = None
        start_node = Node(x=start[0],y=start[1],goal=goal)
        #print(f'start node is {start_node.position()}')
        openset[start_node.position()] = start_node
        heapq.heappush(heap,(start_node.f,start_node))

        # Controls the amount of max iterations before cancel
        #maxiter = 15000
        currentlist = []
        iter = 0
        goal_tolerance = 0.1
        
        while heapq and iter < self.iterations:
            current_pos = min(openset, key=lambda cordinates: openset[cordinates].f)
            try:
                current = heapq.heappop(heap)
            except IndexError:
                return False,[openset[start_node.position()]]
            current = current[1]

            openset.pop(current_pos)
            

            currentlist.append(current.position())
            if abs(current.x-current.goal[0]) <= goal_tolerance and abs(current.y-current.goal[1]) <= goal_tolerance:
               # print(f'diff x {current.x-current.goal[0]}, diffy {current.y-current.goal[1]}')
                #print(f'Path planner: found path {current.position()}')
                #print(f'Path planner: found path {currentlist}')
                return True,self.reconstruct_path(current)
                
            
            closedset[start_node.position()] = start_node
            
            neighbours = self.generate_neighbours(current)
            #rospy.loginfo(f'\nPath planner:neighbours = {neighbours}')
            
            for neighbour in neighbours:
                neighbournode= neighbours[neighbour]

                if neighbournode.position() in closedset:
                    continue
                else:
                    closedset[neighbournode.position()] = neighbournode
                
                if neighbournode.position() in openset:
                    sameCoords = {key: openset[key] for key in openset.keys()
                                        & {(neighbournode.x, neighbournode.y)}}
                    nodes = sameCoords.values()
                    #print(f'nodes = {nodes}')
                    #print(nodes)
                    for node in nodes:
                        if neighbournode.g >= node.g:
                            pass
                        else:
                            heapq.heappush(heap,(neighbournode.f,neighbournode))
                            openset[neighbournode.position()] = neighbournode
                else: 
                    openset[neighbournode.position()] = neighbournode
                    heapq.heappush(heap,(neighbournode.f,neighbournode))
            iter +=1
            #print(iter)

        return False, self.reconstruct_path(current)
    
    def path_smoothing(self,path):
        # check 3 points at a time
        # if the change in dx and dy is the same, remove the middle point
        # if the change in dx and dy is not the same, keep the middle point
        path_length = len(path)
        #print(f'path_length = {path_length}')
        points_to_remove = []
        for point in range( 1, path_length-1):
            #print(f'point = {path[point]}')
            dx1 = (path[point][0] - path[point-1][0])
            dy1 = (path[point][1] - path[point-1][1])
            dx2 = (path[point+1][0] - path[point][0])
            dy2 = (path[point+1][1] - path[point][1])
            #print(f'dx1 = {dx1}, dy1 = {dy1}, dx2 = {dx2}, dy2 = {dy2}')
            if dx1 == dx2 and dy1 == dy2:
                points_to_remove.append(point)
        points_to_remove.reverse()

        #print(f'points_to_remove = {points_to_remove}')
        for point in points_to_remove:
            path.pop(point)
        #print(f'smoothened path = {path}')
        path.pop(0) # remove the first starting point 
        return path


##################################################    MAIN CLASS   ##########################################

class Path_Planner():
    def __init__(self) -> None:
        rospy.init_node('path_planner')
        rospy.sleep(10)
        #server
        #self.client = actionlib.SimpleActionClient('path_tracker', mb.MoveBaseAction)
        #print('waiting for server')
        #self.client.wait_for_server(rospy.Duration(0.5))
        #print('server found')

        #subscribers
        # TODO: change to /test/goal for pathing_brain_tester, else use /move_base_simple/goal
        self.sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        #publishers
        self.viz_path_pub = rospy.Publisher('/viz_path', Path, queue_size = 100)
        self.path_pub = rospy.Publisher('/path',Path,queue_size=100)

        self.start = PoseStamped()
        self.goal = PoseStamped()
        self.goal_local = PoseStamped()
        self.has_recived_goal = False
        
        self.reached_goal = False

        # checks to avoid oscillations
        self.previous_path = []
        self.prev_previous_path = []
        self.path = None
        self.path_smooth = None
        
        # might need to change in the future
        self.rate = rospy.Rate(1)

        self.path_planner = A_star()
        self.last_msg = PoseStamped()

        # services
        self.toggle_uninflation_service = rospy.Service('/path_planner/toggle_uninflation', BoolSetter, self.toggle_uninflation_callback)
        self.is_uninflating_service = rospy.Service('/path_planner/is_uninflating', BoolSrv, self.is_uninflating_callback)
        
        #print('path planner node started')
        rospy.loginfo('path planner node initialized')
        
        #rospy.roslog('path planner node started')
############################################ Callbacks ############################################

    def toggle_uninflation_callback(self, req):
        self.path_planner.should_uninflate_around_goal = req.value
        return BoolSetterResponse()
    
    def is_uninflating_callback(self, req):
        return BoolSrvResponse(self.path_planner.should_uninflate_around_goal)

    def start_and_goal_callback(self,msg):
        self.goal = msg
        self.has_recived_goal = True
        #self.main()
        
    def goal_reached_callback(self,msg):
        self.reached_goal = msg.data

    # does the initial calculation 
    def goal_callback(self, goal):
        self.goal = goal
        self.has_recived_goal = True
        self.status, self.path = self.path_planner.path(self.goal)
        self.path_smooth = self.path_planner.path_smoothing(self.path)
    # print(self.path_smooth )
        self.send_path2(self.path_smooth )

            
############################################ Main ############################################
    
    def local_planner(self):
        # needs to check if new information

        # First point calculations.
        # does local planning but with the fixed point
        Fixedpoint = [self.path[0]]
        if self.path_planner.obstacle_on_calc_path(Fixedpoint,self.path):
            self.status,self.path = self.path_planner.path(self.goal)
            self.path_smooth = self.path_planner.path_smoothing(self.path)
            self.send_path2(self.path_smooth)
        else: 
            # Start is the first point, which is fixed
        # Recalculates path around object to start at next point from global path planner
            if self.path_planner.obstacle_on_calc_path(self.path,self.path_smooth):
                rospy.loginfo(f'is this always triggering?')
                self.status,self.path = self.path_planner.path(self.goal,Fixedpoint)
                """if not self.status:
                    rospy.logerr('Path planner: Could not find path, check submitted goal') """
                self.path.insert(0,Fixedpoint[0])
                self.path_smooth = self.path_planner.path_smoothing(self.path)

                self.send_path2(self.path_smooth)
        

    def calc_orentation_for_points(self, path):
        path_length = len(path)
        point_orientation = []    
        for point in range(1, path_length):
            dx = (path[point][0] - path[point-1][0])
            dy = (path[point][1] - path[point-1][1])
            yaw = math.atan2(dy, dx)
            # TODO needs to be checked if even correct, cannot do from home computer
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            point_orientation.append(q)
        return point_orientation
    
    
    def tranform_path_to_posestamped(self,path):
        path_list = []

        
        # get orientation for each point in relation to the next point
        # and should therefor should the robot always point at next point
        orientations = self.calc_orentation_for_points(path)
        #print(f'orientation {len(orientations)}')
        n=len(path)
        
        for i,point in enumerate(path):
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0

            # dont know why -2 works 
            if i<=n-2:
                pose.pose.orientation.x = orientations[i][0]
                pose.pose.orientation.y = orientations[i][1]
                pose.pose.orientation.z = orientations[i][2]
                pose.pose.orientation.w = orientations[i][3]
            else:
                pose.pose.orientation = self.goal.pose.orientation
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            path_list.append(pose)

        #end = path_list[-1]
        #end.pose.orientation = self.goal.pose.orientation
        #path_list[-1] = end
        #path_tosend.poses = path_list
        if len(path_list) > 0:
            path_list.pop() # replace with goal
        path_list.append(self.goal)
        
        return path_list
    
        
    def send_path2(self,path):
        path_list = self.tranform_path_to_posestamped(path)
        #path_list.pop(0) # so it doenst go to the start point
        #esures that the paths are saved
        if self.previous_path:
                self.prev_previous_path = self.previous_path
                self.previous_path = path_list
        else:
            self.previous_path = path_list
            
        # should solve the oscilation problem but cannot test
        # first iteration 
        if self.prev_previous_path == path_list:
            self.path = self.previous_path
        
        if self.reached_goal:
            
            #print(f'Path planner: has reached goal = {self.reached_goal}')
            self.reached_goal = False
            
        #self.move_to_pub.publish(path_list[0])
        
        # Publishes the path in Rviz
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        for point in path_list:
            path.poses.append(point)

        if path_list:
            # print(path)
            self.viz_path_pub.publish(path)
            self.path_pub.publish(path)
        else: 
            rospy.logerr('Path planner: No path created')
    
    def find_goal(self):
        self.nearest_goal = None
        self.cells = []

        for i , data_i in enumerate(self.path_planner.map_coords):
            for j, data_j in enumerate(data_i.data):
                if int(data_j) == 0:
                    self.cells.append([i,j])
        
        if len(self.cells) > 0:
            self.cells = np.array(self.cells)
            distances = self.cells - self.path_planner.get_start_in_map() # might be problem
            distances = np.linalg.norm(distances, axis=1)
            new_distances = []
            for distance in distances:
                if distance > np.sqrt(8):
                    new_distances.append(distance)
            distances = np.array(new_distances)
            min_distance_index = np.argmin(distances)
            
            self.nearest_goal = self.cells[min_distance_index]

    def in_wall_manager(self):
        if self.path_planner.get_value_of_pos(self.path_planner.get_start_in_map()) >0.8:
            # use exporer to find the closest point to the robot
            self.find_goal()
            if self.nearest_goal is not None:
                self.send_path2([self.nearest_goal])
                return True
        return False


    

    def main(self):
        
        
        if self.path_planner.map_coords == None:
            rospy.logwarn('Path planner: No map yet')
            return
        if self.has_recived_goal == False:
            rospy.logwarn('Path planner: No goal yet')
            return
        start = self.path_planner.get_robot_pose_in_map()
        rospy.loginfo(f'Path planner: start {start.pose.position.x} {start.pose.position.y}')
        #status,self.path = self.path_planner.path(self.goal)
        pos_in_map = self.path_planner.get_start_in_map()
        if not self.status and not self.path:
            rospy.logwarn('Path planner: Failed to find path')
            return
        # If we are inside a wall. Just go to the nearest point that has a 0 value
        if self.path_planner.get_value_of_pos(pos_in_map[0],pos_in_map[1]) >0.8:
            # use exporer to find the closest point to the robot
            self.find_goal()
            if self.nearest_goal is not None:
                self.send_path2([start,self.nearest_goal])
        else:
            self.local_planner()
        #print(f'Path planner: status {self.status}')
        #print(f'Path planner: Goal {self.goal.pose}')
        
        #print('path sent')


    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            #print('spinning like a cat')
            #rospy.sleep(0.5)
            self.rate.sleep()


if __name__ == "__main__":
    path_planner = Path_Planner()
    path_planner.spin()




