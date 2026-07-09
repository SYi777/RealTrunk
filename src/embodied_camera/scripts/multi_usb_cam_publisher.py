#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import threading

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def list_usb_cameras():

    # 执行命令并获取输出
    result = subprocess.run(['v4l2-ctl', '--list-devices'], stdout=subprocess.PIPE, text=True)

    # 获取输出文本
    output = result.stdout

    # 使用正则表达式查找 USB Camera 的设备
    usb_camera_devices = []
    lines = output.splitlines()
    is_usb_camera_section = False

    for line in lines:
        line = line.strip()
        if "USB Camera" in line or "usb camera" in line:  # 检测到 USB Camera 的标题行
            is_usb_camera_section = True
        elif line.startswith('/dev/'):  # 检测到设备路径行
            if is_usb_camera_section:
                usb_camera_devices.append(line)  # 添加设备路径到列表
                is_usb_camera_section = False  # 重置标志以防止错误匹配

    for idx, cam in enumerate(usb_camera_devices):
        rospy.loginfo(f"{idx}: {cam}")

    return usb_camera_devices


def get_camera_params():

    frame_rate = rospy.get_param('~usb_camera_frame_rate', 30)
    width = rospy.get_param('~usb_camera_resolution/width', 640)
    height = rospy.get_param('~usb_camera_resolution/height', 480)
    
    return frame_rate, width, height

def publish_camera_stream(camera_info, frame_rate, width, height,index):

    device_node = camera_info
    cap = cv2.VideoCapture(device_node)
    cap.set(cv2.CAP_PROP_FPS, frame_rate)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    pub = rospy.Publisher(f"/camera{index}/image_raw", Image, queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(frame_rate)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(img_msg)
        else:
            rospy.logerr(f"无法读取摄像头{device_node}的帧")
        rate.sleep()

if __name__ == '__main__':
    
    rospy.init_node('usb_camera_node')
    cameras = list_usb_cameras()
    if not cameras:
        rospy.logerr("未检测到指定的USB摄像头。")
    else:
        frame_rate, width, height = get_camera_params()
        # idx = int(rospy.get_param('~idx', 2))

        # rospy.loginfo(f'idx is :{idx}')

        # if idx == 2:
        threads = []
        for i in range(2):
            cam = cameras[i]
            t = threading.Thread(target=publish_camera_stream, args=(cam, frame_rate, width, height,i))
            t.start()
            threads.append(t)
        for t in threads:
            t.join()
   
        
