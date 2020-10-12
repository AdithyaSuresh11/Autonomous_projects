#!/usr/bin/env python

import cv2
import numpy as np
import sys
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import rospy
import numpy as np
import argparse
import imutils
import cv2

class Image_Stitching():
    def __init__(self) :
        self.ratio=0.5
        self.min_match= 3
        self.sift=cv2.xfeatures2d.SIFT_create()
        self.smoothing_window_size= 1100
    
    def registration(self,cv_image_left,cv_image_right):
        kp1, des1 = self.sift.detectAndCompute(cv_image_left, None)
        kp2, des2 = self.sift.detectAndCompute(cv_image_right, None)
        matcher = cv2.BFMatcher()
        raw_matches = matcher.knnMatch(des1, des2, k=2)
        good_points = []
        good_matches=[]
        for m1, m2 in raw_matches:
            if m1.distance < self.ratio * m2.distance:
                good_points.append((m1.trainIdx, m1.queryIdx))
                good_matches.append([m1])
        img3 = cv2.drawMatchesKnn(cv_image_left, kp1, cv_image_right, kp2, good_matches, None, flags=2)
        cv2.imwrite('matching.jpg', img3)
        # cv2.imshow("Final Result",img3)
        # cv2.waitKey(1)
        if len(good_points) > self.min_match:
            image1_kp = np.float32(
                [kp1[i].pt for (_, i) in good_points])
            image2_kp = np.float32(
                [kp2[i].pt for (i, _) in good_points])
            H, status = cv2.findHomography(image2_kp, image1_kp, cv2.RANSAC,5.0)
        return H

    def create_mask(self,cv_image_left,cv_image_right,version):
        height_cv_image_left = cv_image_left.shape[0]
        width_cv_image_left = cv_image_left.shape[1]
        width_cv_image_right = cv_image_right.shape[1]
        height_panorama = height_cv_image_left
        width_panorama = width_cv_image_left +width_cv_image_right
        offset = int(self.smoothing_window_size / 2)
        barrier = cv_image_left.shape[1] - int(self.smoothing_window_size / 2)
        # print('Width left',width_cv_image_left)
        # print('\n')
        # print('Barrier',barrier)
        # print('Height of left',height_cv_image_left)
        # print('\n')
        # print('Panorama width', width_panorama)
        # print('\n')
        # print('Offset',offset)

        mask = np.zeros((height_panorama, width_panorama))
        if version== 'left_image':
            mask[:, barrier - offset:barrier + offset ] = np.tile(np.linspace(1, 0, 2 * offset ).T, (height_panorama, 1))
            mask[:, :barrier - offset] = 1
        else:
            mask[:, barrier - offset :barrier + offset ] = np.tile(np.linspace(0, 1, 2 * offset ).T, (height_panorama, 1))
            mask[:, barrier + offset:] = 1
        # print(cv2.merge([mask, mask, mask]))
        # print('===================================')
        return cv2.merge([mask, mask, mask])

    def blending(self,cv_image_left,cv_image_right):
        H = self.registration(cv_image_left,cv_image_right)
        height_cv_image_left = cv_image_left.shape[0]
        # print('Height of left',height_cv_image_left)
        # print('\n')
        width_cv_image_left = cv_image_left.shape[1]
        width_cv_image_right = cv_image_right.shape[1]
        height_panorama = height_cv_image_left
        width_panorama = width_cv_image_left +width_cv_image_right
        # print('Panorama width', width_panorama)

        panorama1 = np.zeros((height_panorama, width_panorama, 3))
        # print(panorama1)
        mask1 = self.create_mask(cv_image_left,cv_image_right,version='left_image')
        # print(mask1)
        panorama1[0:cv_image_left.shape[0], 0:cv_image_left.shape[1], :] = cv_image_left
        panorama1 *= mask1
        mask2 = self.create_mask(cv_image_left,cv_image_right,version='right_image')
        panorama2 = cv2.warpPerspective(cv_image_right, H, (width_panorama, height_panorama))*mask2
        result=panorama1+panorama2
        # print(result)
        # print('===================================')
        rows, cols = np.where(result[:, :, 0] != 0)
        min_row, max_row = min(rows), max(rows) + 1
        min_col, max_col = min(cols), max(cols) + 1
        final_result = result[min_row:max_row, min_col:max_col, :]
        # print(final_result)
        # print('===================================')
        return final_result

def left_camera_callback(data):

    global cv_image_left

    # print(data.data)

    try:
        cv_image_left = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
            print(e) 

    height, width, channels = cv_image_left.shape
    hsv = cv2.cvtColor(cv_image_left, cv2.COLOR_BGR2HSV)
    # cv_image_left = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("Image Left", cv_image_left)
    # cv2.waitKey(1)
    # print(cv_image_left)
    return cv_image_left

def right_camera_callback(data):

    global cv_image_left, cv_image_right

    try:
        cv_image_right = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e) 

    height, width, channels = cv_image_right.shape
    hsv = cv2.cvtColor(cv_image_right, cv2.COLOR_BGR2HSV)
    # cv_image_right = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)

    # print(cv_image_left)
    # cv2.imshow("Image Right", cv_image_right)
    # cv2.waitKey(1)
    # final=Image_Stitching().blending(cv_image_left,cv_image_right)
    # cv2.imwrite('panorama.jpg', final)
    # cv2.imshow("Final Result",final)
    # cv2.waitKey(1)
    # print(cv_image_right)
    return cv_image_right

def camera_callback(data):

    # print('Loop')
    global cv_image_left, cv_image_right
    # print(cv_image_left)
    final=Image_Stitching().blending(cv_image_left,cv_image_right)
    cv2.imwrite('panorama.jpg', final)
    

if __name__ == '__main__':

    global cv_image_left, cv_image_right    

    rospy.init_node('Stitching_node', anonymous= True)

    bridge_object = CvBridge()

    left_image_sub = rospy.Subscriber("/left/image_raw",Image,left_camera_callback)
    right_image_sub = rospy.Subscriber("/right/image_raw",Image,right_camera_callback)
    new_sub = rospy.Subscriber("/right/image_raw",Image,camera_callback)

    rate = rospy.Rate(7) # in Hz

    while not rospy.is_shutdown():
        rate.sleep()
    