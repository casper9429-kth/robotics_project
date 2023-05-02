#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
import tf2_ros
import tf2_geometry_msgs # This package has magical side effects, don't remove this import
from visualization_msgs.msg import Marker
from rospy import Subscriber, Service, Publisher
from math import inf
from mapping.srv import CheckPolygon, CheckPolygonResponse, CheckPolygonRequest


class Polygon:
    def __init__(self, corner_points: PoseArray):
        self.corner_points = corner_points

    def is_inside(self,x,y):
        """Check if point is inside polygon"""
        corners = [(point.position.x, point.position.y) for point in self.corner_points.poses]
        n = len(corners)
        inside = False

        p1x, p1y = corners[0]
        for i in range(n+1):
            p2x, p2y = corners[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x,p1y = p2x,p2y

        return inside


class WorkspaceManager:
    def __init__(self) -> None:
        rospy.init_node('workspace_manager')
        
        # Publisher
        self.message_pub_alt = Publisher("/geofence/pose_array", PoseArray, queue_size=10) # TODO: find out why this exists
        self.message_pub = Publisher("/workspace_poses/pose_array", PoseArray, queue_size=10)
        self.marker_pub = Publisher("/workspace_poses/marker", Marker, queue_size=10)
        
        # Services
        self.is_inside_service = Service("/workspace/is_inside", CheckPolygon, self.is_inside_callback)

        # Define rate
        self.update_rate = 1 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate)
        
        # Parameters
        self.pose_array = self.read_poses()
        self.polygon = Polygon(self.pose_array)
        
    def is_inside_callback(self, req: CheckPolygonRequest):
        result = self.polygon.is_inside(req.x, req.y)
        return CheckPolygonResponse(result)

    def is_valid_decimal(self,num):
        try:
            float(num)
        except ValueError:
            return False
        else:
            return True
        
    def read_poses(self):
        pose_array = PoseArray()
        filepath = '/home/robot/dd2419_ws/src/mapping/src/workspace.tsv'
        with open(filepath) as file:
            for line in file:
                value = line.strip().split('\t')
                point = Pose()

                if self.is_valid_decimal(value[0]) and self.is_valid_decimal(value[1]):
                    point.position.x = float(value[0])
                    point.position.y = float(value[1])
                    pose_array.poses.append(point)
                
        pose_array.header.frame_id = 'map'
        return pose_array
        
    def visualize_point(self):
        poselist = [(pose.position.x, pose.position.y, pose.position.z) for pose in self.pose_array.poses]

        mark = Marker()
        mark.header.frame_id = "map"
        mark.header.stamp = rospy.Time.now()
        mark.type = mark.LINE_STRIP
        mark.action = mark.ADD
        mark.scale.x = 0.01
        mark.color.a = 1.0
        mark.color.r = 0.0
        mark.color.g = 1.0
        mark.color.b = 0.0
        mark.pose.orientation.w = 1.0
        mark.pose.position.x = 0.0
        mark.pose.position.y = 0.0
        mark.pose.position.z = 0.0

        mark.id = 0
        mark.lifetime = rospy.Duration(0)
        mark.points = [Point(x=x, y=y, z=z) for x, y, z in poselist]
        mark.points.append(Point(x=poselist[0][0], y=poselist[0][1], z=poselist[0][2]))

        self.marker_pub.publish(mark)

    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        
        self.message_pub.publish(self.pose_array)
        self.message_pub_alt.publish(self.pose_array)

        self.visualize_point()

    def run(self):
        """
        Run the node. 
        Don't change anything here, change main instead.
        """
        
        # Run as long as node is not shutdown
        try:
            while not rospy.is_shutdown():
                self.main()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    node = WorkspaceManager()
    node.run()
