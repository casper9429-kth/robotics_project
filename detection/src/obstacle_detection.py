#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
import numpy as np
import matplotlib.cm

class Obstacle_detection():
    def __init__(self):
        """ Put the node name here, and description of the node"""
        rospy.init_node('Obstacle_Detection')

        # Subscribers 
        self.sub_cloud = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.cloud_callback)
  
        
        # Publisher
        self.pub = rospy.Publisher('/detection/obstacles_pcd', PointCloud2, queue_size=1)

        # Paramethers HERE
        self.cloud = None

 

    def cloud_callback(self, msg: PointCloud2):
        self.cloud = msg
        
        # Convert ROS -> Open3D
        cloud = o3drh.rospc_to_o3dpc(msg)

        #Crop according to BB TODO

        cropped = cloud.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=np.array([-100, -100, -100]), max_bound=np.array([100, 0.075, 100])))

        # Downsample the point cloud to 1 cm
        ds_cloud = cropped.voxel_down_sample(voxel_size=0.01)

        # # Convert Open3D -> NumPy
        # points = np.asarray(ds_cloud.points)
    
        labels = np.array(ds_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=False))
        max_label = labels.max()
        # colors = matplotlib.cm.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        # colors[labels < 0] = 0
        # ds_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

        out_msg = o3drh.o3dpc_to_rospc(ds_cloud)
        out_msg.header = msg.header
        self.pub.publish(out_msg)
        


    def compute_color_pc(self, x,y,z, category_id):
        # Convert ROS -> Open3D
        cloud = o3drh.rospc_to_o3dpc(self.cloud)

        #Crop according to BB TODO

        cropped = cloud.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=np.array([x-0.2, y-0.2, z-0.2]), max_bound=np.array([x+0.2, y+0.2, z+0.2])))

        # Downsample the point cloud to 1 cm
        ds_cloud = cropped.voxel_down_sample(voxel_size=0.01)
        #ds_cloud = cropped.voxel_down_sample(voxel_size=0.01)

        # # Convert Open3D -> NumPy
        points = np.asarray(ds_cloud.points)
        colors = np.asarray(ds_cloud.colors)
    

        
        # Filter Colors

        # filter_r, filter_b, filter_g, filter_w = [], [], [], []
        # count_red, count_blue, count_green, count_wooden = 0,0,0,0
        # for color in colors:
        # # if the element is red, green or blue, set the value to True, otherwise False:
        #     if color[0] > 0.75 and color[0] > (color[1] + color[2]): 
        #     #if color[0] > 1.2*(color[1] + color[2]) and color[0] > 0.75 : 
        #         filter_r.append(True)
        #         filter_g.append(False)
        #         filter_b.append(False)
        #         filter_w.append(False)
                
        #         count_red+=1
            
        #     elif color[1] > 0.5*(color[0] + color[2]) and color[1] > 0.6:
        #     # elif color[1] > (color[0] + color[2]) and color[1] > 0.4:
        #         filter_r.append(False)
        #         filter_g.append(True)
        #         filter_b.append(False)
        #         filter_w.append(False)
                
        #         count_green+=1
            
        #     elif color[2] > (color[0] + color[1])and color[1] > 0.5:
        #         filter_r.append(False)
        #         filter_g.append(False)
        #         filter_b.append(True)
        #         filter_w.append(False)
                
        #         count_blue+=1
        #     elif color[1] > (1.2*color[2]) and color[0] > 0.75: # TODO: wooden color to define!
        #         filter_r.append(False)
        #         filter_g.append(False)
        #         filter_b.append(False)
        #         filter_w.append(True)
        #         count_wooden+=1
        #     else:
        #         filter_r.append(False)
        #         filter_g.append(False)
        #         filter_b.append(False)
        #         filter_w.append(False)
        
        # max_color = max(count_red, count_green, count_blue, count_wooden)
        # mapping = None
        # if category_id == 6:
        #     mapping = self.sub_mapping_cube
        # else:
        #     mapping = self.sub_mapping_sphere

        # category_name = ""
        # if max_color == 0:
        #     return None

        # if max_color == count_red:
        #     rospy.loginfo("RED Object detected")
        #     category_name = mapping[0]
        #     ds_cloud.points = o3d.utility.Vector3dVector(points[filter_r])
        #     ds_cloud.colors = o3d.utility.Vector3dVector(colors[filter_r])
        #     # Convert Open3D -> ROS
        #     out_msg = o3drh.o3dpc_to_rospc(ds_cloud)
        #     out_msg.header = self.cloud.header
        #     self.pub.publish(out_msg)
        # elif max_color == count_green:
        #     rospy.loginfo("GREEN Object detected")
        #     category_name = mapping[1]
        #     ds_cloud.points = o3d.utility.Vector3dVector(points[filter_g])
        #     ds_cloud.colors = o3d.utility.Vector3dVector(colors[filter_g])
        #     # Convert Open3D -> ROS
        #     out_msg = o3drh.o3dpc_to_rospc(ds_cloud)
        #     out_msg.header = self.cloud.header
        #     self.pub.publish(out_msg)
        # elif max_color == count_blue:
        #     rospy.loginfo("BLUE Object detected")
        #     category_name = mapping[2]
        #     ds_cloud.points = o3d.utility.Vector3dVector(points[filter_b])
        #     ds_cloud.colors = o3d.utility.Vector3dVector(colors[filter_b])
        #     # Convert Open3D -> ROS
        #     out_msg = o3drh.o3dpc_to_rospc(ds_cloud)
        #     out_msg.header = self.cloud.header
        #     self.pub.publish(out_msg)
        # elif max_color == count_wooden and category_id== 6:
        #     rospy.loginfo("WOODEN Object detected")
        #     category_name = mapping[3]
        #     ds_cloud.points = o3d.utility.Vector3dVector(points[filter_w])
        #     ds_cloud.colors = o3d.utility.Vector3dVector(colors[filter_w])
        #     # Convert Open3D -> ROS
        out_msg = o3drh.o3dpc_to_rospc(ds_cloud)
        out_msg.header = self.cloud.header
        self.pub.publish(out_msg)



 
    def run(self):
        """
        Run the node. 
        Don't change anything here, change main instead.
        """
        
        # Run as long as node is not shutdown
        while not rospy.is_shutdown():
            self.main()
            self.rate.sleep()


if __name__ == "__main__":

    obstacle_detector=Obstacle_detection()
    rospy.spin()
    #node.run()




# https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html#statistical-outlier-removal

# https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html#passthrough


# Spatial change detection on unorganized point cloud data
# https://pcl.readthedocs.io/projects/tutorials/en/latest/octree_change.html#octree-change-detection

# Robust pose estimation of rigid objects
# https://pcl.readthedocs.io/projects/tutorials/en/latest/alignment_prerejective.html#alignment-prerejective


# Euclidean Cluster Extraction
# https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html#cluster-extraction


# Min-Cut Based Segmentation
# https://pcl.readthedocs.io/projects/tutorials/en/latest/min_cut_segmentation.html#min-cut-segmentation