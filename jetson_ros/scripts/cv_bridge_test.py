'''
Author: pandy 14674879+pandy2@user.noreply.gitee.com
Date: 2024-07-28 16:43:42
LastEditors: pandy 14674879+pandy2@user.noreply.gitee.com
LastEditTime: 2024-08-25 20:25:43
FilePath: /robot-zero/src/Jetson_ros/script/cv_bridge_test.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!/usr/bin/env python
from __future__ import print_function

import roslib
# import rosbag
import sys
import rospy
import cv2
import numpy as np
import open3d as o3d

from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from orbbec_camera.srv import SetBool, GetBool, SetString, GetString, SetInt32, GetInt32





class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

    self.bridge = CvBridge()
    # self.color_image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.color_callback)
    # self.ir_image_sub = rospy.Subscriber("/camera/ir/image_raw",Image,self.ir_callback)
    # self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.depth_callback)
    self.points_sub = rospy.Subscriber("/camera/depth/points",PointCloud2,self.points_callback)
    
    self.laser_server = rospy.ServiceProxy("/camera/set_laser", SetBool)
    self.floor_server = rospy.ServiceProxy("/camera/set_floor", SetBool)
    self.ldp_server = rospy.ServiceProxy("/camera/set_ldp", SetBool)
    self.switch_ir_server = rospy.ServiceProxy("/camera/switch_ir", SetBool)

    self.left_ir  = True

    # self.bag = rosbag.Bag("/home/rosdemo/demo/test.bag",'w')

  def color_callback(self, image):
    try:
      cv_color_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("color image", cv_color_image)
    cv2.waitKey(3)
    # cv2.imwrite("color_image_raw.png", cv_image)
    # plt.imshow(cv_image, cmap = 'gray')
    # plt.show()
    cv2.destroyAllWindows()


  def ir_callback(self,image):
    try:
      cv_ir_image = self.bridge.imgmsg_to_cv2(image, "mono16")
    except CvBridgeError as e:
      print(e)

    cv_ir_image.byteswap(True)
    cv2.imshow("ir_image", cv_ir_image)
    cv2.waitKey(3)

    # cv2.imwrite("ir_image_left.png", cv_ir_image)
    cv2.imwrite("ir_image_right.png", cv_ir_image)

    cv2.destroyAllWindows()

    # self.ldp_server.wait_for_service()

    # resp1 = self.ldp_server(False)

    # resp2 = self.laser_server(False)

    # resp3 = self.floor_server(True)

    # if(resp3):
    #   self.left_ir = False

    # if(resp3):
    #   print("open floor success\n")
    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

  def depth_callback(self, image):
    try:
      cv_depth_image = self.bridge.imgmsg_to_cv2(image, "16UC1")
    except CvBridgeError as e:
      print(e)
    
    cv_depth_image = cv_depth_image*255

    cv2.imshow("depth_image", cv_depth_image)
    cv2.waitKey(3)
    cv2.imwrite("depth_image_raw.png", cv_depth_image)
    cv2.destroyAllWindows()

  def points_callback(self, msg):
    assert isinstance(msg, PointCloud2)

    points = point_cloud2.read_points(msg,field_names=("x","y","z"))
    #points = point_cloud2.read_points_list(
    #    msg, field_names=("x", "y", "z"))

    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([p])
    print(p)
    

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)