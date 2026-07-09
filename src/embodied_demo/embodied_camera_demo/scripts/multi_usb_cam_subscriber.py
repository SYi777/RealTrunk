#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber:
    
    def __init__(self):
        
        self.bridge = CvBridge()

        # 获取已发布的摄像头索引列表
        idx = rospy.get_param('~idx', None)
        if idx is None:
            rospy.logerr("未获取到已发布的摄像头id。")
            return
        
        if idx != 2:

            topic_name = f"/camera{idx}/image_raw"
            rospy.loginfo(f"订阅主题：{topic_name}")
            sub = rospy.Subscriber(topic_name, Image, self.image_callback, callback_args=idx)

            rospy.spin()
        
        else:
            topic_name0 = f"/camera{0}/image_raw"
            topic_name1 = f"/camera{1}/image_raw"
            sub0 = rospy.Subscriber(topic_name0, Image, self.image_callback, callback_args=0)
            sub1 = rospy.Subscriber(topic_name1, Image, self.image_callback, callback_args=1)

            rospy.spin()

    def image_callback(self, msg, camera_index):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 显示图像，窗口名称为摄像头索引

            cv2.imshow(f"usb camera {camera_index}", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"图像转换失败：{e}")

if __name__ == '__main__':

    rospy.init_node('camera_subscriber_node')
    CameraSubscriber()

