#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import numpy as np

class ImagePublisher:
    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)
        
        # 初始化CvBridge
        self.bridge = CvBridge()
        
        # 创建六个发布者，分别发布到不同的topic
        self.publishers = [
            rospy.Publisher('/cam_front/raw', Image, queue_size=10),
            rospy.Publisher('/cam_front_left/raw', Image, queue_size=10),
            rospy.Publisher('/cam_front_right/raw', Image, queue_size=10),
            rospy.Publisher('/cam_back/raw', Image, queue_size=10),
            rospy.Publisher('/cam_back_left/raw', Image, queue_size=10),
            rospy.Publisher('/cam_back_right/raw', Image, queue_size=10)
        ]
        
        # 加载六张图片
        self.images = [
            cv2.imread("/home/itachi/project/catkin_ws_test/src/pcd_publisher/scripts/2_FRONT.jpg")
            , cv2.imread("/home/itachi/project/catkin_ws_test/src/pcd_publisher/scripts/2_FRONT_LEFT.jpg")
            , cv2.imread("/home/itachi/project/catkin_ws_test/src/pcd_publisher/scripts/2_FRONT_RIGHT.jpg")
            , cv2.imread("/home/itachi/project/catkin_ws_test/src/pcd_publisher/scripts/2_BACK.jpg")
            ,cv2.imread("/home/itachi/project/catkin_ws_test/src/pcd_publisher/scripts/2_BACK_LEFT.jpg")
            , cv2.imread("/home/itachi/project/catkin_ws_test/src/pcd_publisher/scripts/2_BACK_RIGHT.jpg")
        ]
        
        # 检查图片是否加载成功
        for i, img in enumerate(self.images):
            if img is None:
                rospy.logerr(f"Failed to load image_{i}.jpg")
                rospy.signal_shutdown("Failed to load images")
        
        # 设置发布频率
        self.rate = rospy.Rate(10)  # 10Hz

    def publish_images(self):
        while not rospy.is_shutdown():
            for i, img in enumerate(self.images):
                if i != 0:
                    img = np.zeros_like(img)
                # 将OpenCV图像转换为ROS图像消息
                ros_image = self.bridge.cv2_to_imgmsg(img, "bgr8")
                ros_image.header.stamp = rospy.Time.now()
                
                # 发布图像消息
                self.publishers[i].publish(ros_image)
                rospy.loginfo(f"Published image_{i}.jpg to /camera/image_raw_{i}")
            
            # 按照设定的频率休眠
            self.rate.sleep()

if __name__ == '__main__':
    try:
        image_publisher = ImagePublisher()
        image_publisher.publish_images()
    except rospy.ROSInterruptException:
        pass