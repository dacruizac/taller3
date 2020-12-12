#!/usr/bin/python

from __future__ import print_function
import rospy
from std_msgs.msg import Bool,Int8,Float64MultiArray
import numpy as np
from   rospy.numpy_msg import numpy_msg
from tf.transformations import rotation_matrix 

import tf2_ros
import tf_conversions

from geometry_msgs.msg import TransformStamped,Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

from path_planner import planner_algorithm

import sys

class node():
  def __init__(self,pos,prev):
    self.prev=prev
    self.pos=pos


class planning_controller(object):
    def __init__(self):
        #self.flag = False
        self.init_vars()
        self.init_subscribers()
        self.init_publishers()
        self.initial_angle()
        self.init_first_TM(self.ang)
        if (not self.initial_calibration()):
            return

        #self.rate=rospy.Rate(10)
        self.bridge = CvBridge()

        while not rospy.is_shutdown():
            if self.flag:
                self.get_odom()
                self.calc_odom_transf()
                self.map_publish()
            self.rate.sleep()
        pass


    def init_vars(self):
        self.flag=False
        self.path=[]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate=rospy.Rate(10)
        self.actual_pos=np.zeros((2,1),dtype=np.float)
        pass

    def init_subscribers(self):
        self.image_sub = rospy.Subscriber("/map", numpy_msg(OccupancyGrid), self.map_callback,queue_size=10)
        self.destiny_sub = rospy.Subscriber("/goal",numpy_msg(Float64MultiArray),self.goal_callback,queue_size=10)

    def init_publishers(self):
        self.image_pub = rospy.Publisher("/map_image", Image, queue_size=1)
        self.image_pub_dil = rospy.Publisher("/map_image_dil", Image, queue_size=1)
        self.position_pub = rospy.Publisher("/tll3/goal", numpy_msg(Float64MultiArray), queue_size=1)
        self.position_msg = Float64MultiArray()


    def initial_angle(self):
        self.ang=None
        if rospy.has_param("ang"):
            rospy.logwarn("param")
            self.ang=rospy.get_param("ang")
        elif len(sys.argv) >= 2:
            rospy.logwarn("arg")
            self.ang=float(sys.argv[1])
        else:
            rospy.logwarn("default")
            self.ang=np.deg2rad(-90)

    def init_first_TM(self,angle):
        self.original_fr=np.identity(4,dtype=np.float)
        rot=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]
        self.original_fr[0:2,0:2]=np.array(rot,dtype=np.float)
        #print(self.original_fr)

    def initial_calibration(self):
        i=0
        while i<10:
            try:
                self.trans_odom = self.tfBuffer.lookup_transform("odom","base_footprint", rospy.Time())
                i=15
            except:
                i+=1
                self.rate.sleep()
                rospy.logwarn("Error trying to look for transform")
        if (i<15):
            return False
        return True

    def get_odom(self):
        try:
            self.trans_odom = self.tfBuffer.lookup_transform("odom","base_footprint", rospy.Time())
        except:
            rospy.logwarn("Error trying to look for transform")

    def calc_odom_transf(self):
        self.quat = np.array([self.trans_odom.transform.rotation.x, \
                                    self.trans_odom.transform.rotation.y, \
                                    self.trans_odom.transform.rotation.z, \
                                    self.trans_odom.transform.rotation.w])
        self.rot_matr_base_odom = tf_conversions.transformations.quaternion_matrix(self.quat)
        self.rot_matr_base_odom = self.rot_matr_base_odom.copy()
        self.rot_matr_base_odom[0,3]=self.trans_odom.transform.translation.x
        self.rot_matr_base_odom[1,3]=self.trans_odom.transform.translation.y
        self.rot_matr_base_odom[2,3]=self.trans_odom.transform.translation.z

        self.base_2_origin= np.matmul(self.original_fr,self.rot_matr_base_odom)
        
        self.rot_origin_2_base = self.base_2_origin.copy()[0:2,0:2]
        self.rot_origin_2_base=np.transpose(self.rot_origin_2_base)

        self.actual_pos[:,0]=self.base_2_origin[0:2,3]

    def map_publish(self):

        pos_x_robot = self.map.shape[1] - (-((self.actual_pos[0,0]-0.178)/self.map_res) - (self.map_origin.position.x/self.map_res))
        pos_y_robot = self.map.shape[0] - (self.actual_pos[1,0]/self.map_res - (self.map_origin.position.y/self.map_res))
        pos_x_robot = np.uint(pos_x_robot)
        pos_y_robot = np.uint(pos_y_robot)

        cv_image = np.zeros((self.map.shape[0],self.map.shape[1],3), dtype = np.uint8)
        cv_image_dil = np.zeros((self.map.shape[0],self.map.shape[1],3), dtype = np.uint8)
        cv_image[:,:,2] = self.map.copy()
        cv_image_dil[:,:,2] = self.map_dilated.copy()
        cv_image[pos_y_robot,pos_x_robot, 0] = 100 # Hue
        cv_image[pos_y_robot,pos_x_robot, 1] = 255 # Saturation
        cv_image[pos_y_robot,pos_x_robot, 2] = 255 # Value
        cv2.circle(cv_image,(pos_x_robot,pos_y_robot), 8, (100,255,255), 1)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_HSV2BGR)
        for i in self.path:
            cv_image[i[0],i[1], 0] = 100 # Hue
            cv_image[i[0],i[1], 1] = 255 # Saturation
            cv_image[i[0],i[1], 2] = 255 # Value
        try:
            #self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image2))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

        cv_image_dil[pos_y_robot,pos_x_robot, 0] = 100 # Hue
        cv_image_dil[pos_y_robot,pos_x_robot, 1] = 255 # Saturation
        cv_image_dil[pos_y_robot,pos_x_robot, 2] = 255 # Value
        cv2.circle(cv_image_dil,(pos_x_robot,pos_y_robot), 8, (100,255,255), 1)
        for i in self.path:
            cv_image_dil[i[0],i[1], 0] = 100 # Hue
            cv_image_dil[i[0],i[1], 1] = 255 # Saturation
            cv_image_dil[i[0],i[1], 2] = 255 # Value
            #cv2.circle(cv_image_dil,(pos_x_robot,pos_y_robot), 8, (100,255,255), 1)

        cv_image_dil = cv2.cvtColor(cv_image_dil, cv2.COLOR_HSV2BGR)
        try:
            #self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image2))
            self.image_pub_dil.publish(self.bridge.cv2_to_imgmsg(cv_image_dil, "bgr8"))
        except CvBridgeError as e:
            print(e)

        


    def map_callback(self,data):
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_res = data.info.resolution
        self.map_origin = data.info.origin

        self.map = data.data

        self.map = np.uint8(np.resize(self.map, [self.map_height, self.map_width]))
        t,self.map = cv2.threshold(self.map,70,255,cv2.THRESH_BINARY)
        self.map=self.map[::-1,:]
        self.map_dilatation()

        self.create_graph()
        self.flag = True


    def map_dilatation(self):
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3),(0, 0))
        self.map_dilated=cv2.dilate(self.map,element,iterations=8)

    def create_graph(self):
        self.map_graph=np.zeros((self.map_height, self.map_width),dtype=np.uint8)
        for i in range(self.map_height):
            for j in range(self.map_width):
                if self.map_dilated[i,j]==0:
                    if i!=0:
                        if self.map_dilated[i-1,j]==0:
                            self.map_graph[i,j] = self.map_graph[i,j] | 1 << 0
                    if i!=self.map_height-1:
                        if self.map_dilated[i+1,j]==0:
                            self.map_graph[i,j] = self.map_graph[i,j] | 1 << 1
                    if j!=0:
                        if self.map_dilated[i,j-1]==0:
                            self.map_graph[i,j] = self.map_graph[i,j] | 1 << 2
                    if i!=self.map_width-1:
                        if self.map_dilated[i-1,j]==0:
                            self.map_graph[i,j] = self.map_graph[i,j] | 1 << 3
        print(self.map_graph)
        self.path_plan=planner_algorithm(self.map_graph)


    def goal_callback(self,msg):
        pos_x = self.map.shape[1] - (-((msg.data[0]-0.178)/self.map_res) - (self.map_origin.position.x/self.map_res))
        pos_y = self.map.shape[0] - (msg.data[1]/self.map_res - (self.map_origin.position.y/self.map_res))
        print("posx",pos_x)
        print("posy",pos_y)
        pos_x = np.uint(pos_x)
        pos_y = np.uint(pos_y)
        print("posx",type(pos_x))
        print("posy",type(pos_y))
        #self.path=[(pos_x,pos_y)]

        pos_x_robot = self.map.shape[1] - (-((self.actual_pos[0,0]-0.178)/self.map_res) - (self.map_origin.position.x/self.map_res))
        pos_y_robot = self.map.shape[0] - (self.actual_pos[1,0]/self.map_res - (self.map_origin.position.y/self.map_res))
        pos_x_robot = np.uint(pos_x_robot)
        pos_y_robot = np.uint(pos_y_robot)

        print("posx_r",pos_x_robot)
        print("posy_r",pos_y_robot)

        print((pos_y_robot,pos_x_robot))

        final_node,cost=self.path_plan.path_find((pos_y_robot,pos_x_robot),(pos_y,pos_x))

        node=final_node
        list_nodes=[]
        while node!=None:
            list_nodes.insert(0,node.pos)
            node=node.prev

        special_points=[]
        for i in list_nodes:
            if len(special_points)==0:
                special_points.append(i)
            elif special_points[-1][0]!=i[0] and special_points[-1][1]!=i[1]:
                special_points.append(i)
            elif i==list_nodes[-1]:
                special_points.append(i)

        for i in special_points:
            print(self.To_xy(i))
            self.position_msg.data=np.array(self.To_xy(i),dtype=np.float)
            self.position_pub.publish(self.position_msg)
            self.rate.sleep()
        self.path = list_nodes
    
    def To_xy(self,point):
        pos_x=point[1]
        pos_y=point[0]

        pos_x=(pos_x - self.map.shape[1])*self.map_res - self.map_origin.position.x + 0.178
        pos_y=-(pos_y - self.map.shape[0])*self.map_res + self.map_origin.position.y
        return (pos_x,pos_y)
