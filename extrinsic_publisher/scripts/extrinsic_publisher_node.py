#!/usr/bin/env python3

import rospy
import yaml
import os
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
from tf.transformations import quaternion_from_matrix

# Load extrinsics from YAML file
def load_extrinsics(file_path):
    with open(file_path, 'r') as f:
        extrinsics = yaml.safe_load(f)
    return extrinsics['camera_extrinsics']

# Convert 4x4 matrix to translation and quaternion
def matrix_to_transform(matrix):
    # Ensure the matrix is a NumPy array
    matrix = np.array(matrix).reshape((4, 4))
    
    # Extract translation
    translation = matrix[:3, 3]
    
    # Extract rotation matrix and convert to quaternion
    rotation_matrix = matrix[:3, :3]
    quaternion = quaternion_from_matrix(matrix)
    
    return translation, quaternion

# Publish transforms
def publish_transforms(extrinsics):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transforms = []

    lidar_frame_id = rospy.get_param('~lidar_frame_id', 'lidar')
    for camera_name, data in extrinsics.items():
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = lidar_frame_id  
        t.child_frame_id = camera_name

        # Convert 4x4 matrix to translation and quaternion
        matrix = data['matrix']
        translation, quaternion = matrix_to_transform(matrix)

        # Print the matrix, translation, and quaternion
        rospy.loginfo(f"Publishing transform for {camera_name}:")
        rospy.loginfo(f"Original Matrix:\n{np.array(matrix).reshape((4, 4))}")
        rospy.loginfo(f"Translation: {translation}")
        rospy.loginfo(f"Quaternion: {quaternion}")

        # Set translation
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        # Set quaternion
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        transforms.append(t)

    broadcaster.sendTransform(transforms)

if __name__ == "__main__":
    rospy.init_node('extrinsics_publisher', anonymous=True)
    
    # Path to your YAML file
    yaml_file_path = rospy.get_param('~extrinsics_file', 
                                     os.path.join(os.path.dirname(__file__), '..', 'config', 'extrinsics.yaml'))
    extrinsics = load_extrinsics(yaml_file_path)
    
    publish_transforms(extrinsics)
    rospy.spin()