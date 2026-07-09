#!/usr/bin/env python3

import threading

import pyrealsense2 as rs
import numpy as np
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class RealSenseCamera:

    def __init__(self):

        rospy.init_node('realsense_camera_node', anonymous=True)

        self.bridge = CvBridge()
        self.devices = self.list_realsense_devices()

        if not self.devices:
            rospy.logerr("No RealSense devices found.")
            return

        choice_indices = range(len(self.devices))
        try:
            choice_indices = [int(index) for index in choice_indices]
        except ValueError:
            rospy.logerr("Invalid input. Please enter numbers separated by spaces.")
            return

        for index in choice_indices:
            if index not in range(len(self.devices)):
                rospy.logerr(f"No device with number {index} found.")
        self.start_cameras(choice_indices)


    def start_cameras(self, choice_indices):
        """Starts one or multiple RealSense cameras based on provided indices.

            This function initializes RealSense camera pipelines for the specified device indices.
            it starts each camera in a separate thread to handle multiple cameras concurrently. The function uses the device serial numbers
            stored in `self.devices` to select and start the camera pipelines.

            Args:
                choice_indices (list): a list of integer indices corresponding to the devices in `self.devices` to start.

        """
        thread_list = []

        for index in choice_indices:
            device = self.devices[index]
            serial_number = device["serial"]

            try:

                pipeline = self.select_realsense_device(serial_number)

            except Exception as e:
                rospy.logerr(f'd435 camera {serial_number}:{e}')

            else:
                rospy.loginfo(f'serial_number {serial_number} initialized succeed')

                thread_ = threading.Thread(
                    target=self.start_camera_pipeline,
                    args=(pipeline, index)
                )
                thread_.start()
                thread_list.append(thread_)

        # Wait for all threads to finish
        for thread_ in thread_list:
            thread_.join()

    def list_realsense_devices(self):
        """Retrieve a list of RealSense D435 camera devices by serial numbers.

            Reads serial numbers of three D435 cameras (left, right, head) from ROS parameters
            in the global namespace. Returns a list of dictionaries with device numbers and serials
            if all are found, else logs an error and return None.

        Args:
            None .

        Returns:
            list: A list of dictionaries, where each dictionary contains:
                - number (int): The index of the camera (0 for left, 1 for right, 2 for head).
                - serial (str): The serial number of the camera.
                Returns an empty list if any serial number is not found.

        Example:
            >>> camera = RealSenseCamera()
            >>> devices = camera.list_realsense_devices()
            >>> print(devices)
            [{'number': 0, 'serial': '148522071717'}, {'number': 1, 'serial': '152122075333'}, {'number': 2, 'serial': '123456789012'}]

        Notes:
            - The function assumes that the ROS parameters '/d435_camera_serial_numbers/left_d435',
            '/d435_camera_serial_numbers/right_d435', and '/d435_camera_serial_numbers/head_d435'
             are set in the ROS parameter server.
            - If any serial number is not found, an error is logged, and an empty list is returned.
        """

        # Retrieve serial numbers for the three D435 cameras
        serial0 = rospy.get_param('~d435_camera_serial_numbers/left_d435', None)
        serial1 = rospy.get_param('~d435_camera_serial_numbers/right_d435', None)
        serial2 = rospy.get_param('~d435_camera_serial_numbers/head_d435', None)


        rospy.loginfo(f"0:{serial0}")
        rospy.loginfo(f"1:{serial1}")
        rospy.loginfo(f"2:{serial2}")
        
        device_list = []

        # Check if all serial numbers are present
        if serial0 and serial1 and serial2 :
            device_list.append({"number":0,"serial":serial0})
            device_list.append({"number":1,"serial":serial1})
            device_list.append({"number":2,"serial":serial2})

            return device_list
        
        # Log error if any serial number is missing
        rospy.logerr("D435 camera serial number not found in the configuration file.")

        return device_list

    def select_realsense_device(self, serial_number):
        

        """Initialize a RealSense device by serial number.

            Configures and starts a RealSense pipeline with 640x480 color (BGR8) and depth (Z16) streams
            at 15 FPS for the specified device.

            Args:
                serial_number (str): Serial number of the RealSense device.

            Returns:
                rs.pipeline: Initialized pipeline with enabled streams.

            Raises:
                RuntimeError: If the device is not found or fails to start.

            Example:
                >>> pipeline = camera.select_realsense_device("148522071717")
                >>> frames = pipeline.wait_for_frames()
        """
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial_number)


        width = rospy.get_param('~d435_camera_resolution/width', None)
        height = rospy.get_param('~d435_camera_resolution/height', None)
        rate = rospy.get_param('~d435_camera_frame_rate',None)

        # Enable streams
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, rate)
        # Add depth stream if needed
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, rate)

        pipeline.start(config)
        return pipeline

    def start_camera_pipeline(self, pipeline, camera_index):

        """Start streaming and publishing images from a RealSense camera.

            This function continuously captures color and depth frames from the given RealSense pipeline,
            converts them to ROS Image messages, and publishes them to topics specific to the camera index.
            It runs until ROS shutdown, then stops the pipeline.

            Args:
                pipeline (rs.pipeline): Initialized RealSense pipeline for the camera.
                camera_index (int): Index of the camera (e.g., 0 for left, 1 for right).

            Returns:
                None
        """
        
        color_pub = rospy.Publisher(f'/camera_d435_{camera_index}/color/image_raw', Image, queue_size=10)
        depth_pub = rospy.Publisher(f'/camera_d435_{camera_index}/depth/image_raw', Image, queue_size=10)

        try:
            while not rospy.is_shutdown():
                
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                # Process color image
                color_image = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                color_pub.publish(color_msg)

                # Process depth image
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
                depth_pub.publish(depth_msg)

        except rospy.ROSInterruptException:
            pass
        finally:
            pipeline.stop()
            rospy.loginfo(f"Camera {camera_index} stopped.")


if __name__ == "__main__":
    RealSenseCamera()
