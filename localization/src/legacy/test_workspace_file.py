#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray


def is_valid_decimal(num):
    try:
        float(num)
    except ValueError:
        return False
    else:
        return True
workspace_points = []

with open('/home/robot/dd2419_ws/src/localization/src/test_workspace.tsv') as file:
    for line in file:
        value = line.strip().split('\t')
        print(value)
        point = PoseStamped()
        point.header.frame_id = 'map'

        if is_valid_decimal(value[0]) and is_valid_decimal(value[1]):
            point.pose.position.x = float(value[0])
            point.pose.position.y = float(value[1])
            workspace_points.append(point)
#print(workspace_points)

pose_array = PoseArray()
pose_array.header.frame_id = 'map'
pose_array.poses = workspace_points
print(pose_array)