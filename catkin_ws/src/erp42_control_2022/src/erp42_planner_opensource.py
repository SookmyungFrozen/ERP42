#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from re import T
import sys,os
import rospy
import rospkg
import math
import time
import serial

from nav_msgs.msg import Path,Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64,Int16,Float32MultiArray
from control_msgs.msg import Velocity, GPSVelocity, Gear
from geometry_msgs.msg import PoseStamped,Point,Pose, PoseArray, TwistWithCovarianceStamped
from morai_msgs.msg import CtrlCmd, LocalControl, Estop
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from ublox_msgs.msg import NavPVT
from lib.utils_0912_mainopen import pathReader,findLocalPath,purePursuit,pidController,path1Reader,path2Reader,middlepathReader

class erp_planner():
    def __init__(self):
        rospy.init_node('ERP42_planner')
        arg = rospy.myargv(argv = sys.argv)
        self.global_path_name = '1002_main_final_fmtc'  
        path_name_1 = '1002_main_final_fmtc'


        #msg 객체 생성
        way_msg=PoseStamped()
        self.ctrl_msg= CtrlCmd()
        steer_msg = Pose()
        local_control_msg = LocalControl()
        self.pose_msg=Odometry()
        self.curvel_msg=Velocity()
        self.gear_msg=Gear()



        #클래스 객체 생성
        self.pure_pursuit = purePursuit()
        pid = pidController()
        look_steering_point = Point() #steering 계산에 기준이 되는 포인트

        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1)
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        local_control_pub = rospy.Publisher('/local_control', LocalControl, queue_size=1)
        
        #subscriber
        rospy.Subscriber("/odom/filtered", Odometry, self.pose_callback)
        rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.boundboxes_callback)
        rospy.Subscriber("/ERP42_gear", Gear, self.gear_callback)




       #Path 생성에 대한 객체, 변수
        self.path_reader = pathReader('erp42_control_2022')
        #장애물 옆 lane 탐색
        self.middle_path_reader = middlepathReader('erp42_control_2022')
        self.path1_reader = path1Reader('erp42_control_2022')
        self.path2_reader = path2Reader('erp42_control_2022')
        self.global_path = self.path_reader.read_txt(self.path_name+".txt")



        #flag 바깥으로 빼기
        self.fix_brake_mode = False
        self.variable_brake_mode = False
        #전체 주행
        self.global_mode = True

      
        rate = rospy.Rate(10) # 10Hz

        while not rospy.is_shutdown():

            local_path, self.current_waypoint=findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
            target_velocity = self.global_path.poses[self.current_waypoint].pose.position.z #180
            #target_velocity=180
            #pid-속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)
            control_input=pid.pid(target_velocity,self.curvel_msg.velocity) 
            
            print("--------------------------------------------------")
            print("path_name",self.path_name)
            print("current waypoint: ",self.current_waypoint)
            print("global mode?",self.global_mode)
            print("parking mode?",self.parking_mode)
            print("mo mode?",self.mo_mode)
            print("so mode?",self.so_mode)
            print("meagae ? ", self.maegae)
            print("--------------------------------------------------")
    
            #self.pure_pursuit
            self.pure_pursuit.getPath(local_path)
            self.pure_pursuit.getPoseStatus(self.pose_msg)
            self.pure_pursuit.getVelStatus(self.curvel_msg)
            self.pure_pursuit.getYawStatus(self.yaw_msg)

            self.ctrl_msg.seq = self.pose_msg.header.seq
            
            
            local_path, self.current_waypoint=findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
            self.ctrl_msg.steering, look_steering_point, self.target_angle_index = self.pure_pursuit.steering_angle(self.current_waypoint)
            self.ctrl_msg.seq = self.pose_msg.header.seq
 
            if control_input > 0 and control_input <= 200:
                rospy.loginfo(f"[1 ~ 200] if success control_input : {control_input}")
                self.ctrl_msg.accel= control_input
                self.ctrl_msg.brake= 0

            elif control_input > 200:
                rospy.loginfo(f"[201 ~ ] if success control_input : {control_input}")
                self.ctrl_msg.accel= target_velocity
                self.ctrl_msg.brake= 0

            elif control_input < -200:
                rospy.loginfo(f"[back] if success control_input : {control_input}")
                self.ctrl_msg.accel= 0
                self.ctrl_msg.brake= 200

            else :
                self.ctrl_msg.accel= 0
                self.ctrl_msg.brake= -control_input

            self.ctrl_pub.publish(self.ctrl_msg)
            global_path_pub.publish(self.global_path)

            self.control_input=self.ctrl_msg.accel
            self.steering_angle=self.ctrl_msg.steering
            self.control_input=self.ctrl_msg.accel

            #local_control 값 넣어주는 부분
            local_control_msg.seq = self.pose_msg.header.seq
            local_control_msg.gps_x = self.pose_msg.pose.pose.position.x
            local_control_msg.gps_y = self.pose_msg.pose.pose.position.y
            local_control_msg.gps_z = self.pose_msg.pose.pose.position.z
            local_control_msg.velocity = self.curvel_msg.velocity/10
            local_control_msg.gps_velocity = 0
            local_control_msg.waypoint_size = len(self.global_path.poses)
            local_control_msg.cur_waypoint = self.current_waypoint
            local_control_pub.publish(local_control_msg)
            
            # 이거?
            rate.sleep()

    def pose_callback(self,data):
        self.pose_msg=Odometry()
        self.pose_msg=data

    def velocity_callback(self,speed_data):
        self.curvel_msg=Velocity()
        self.curvel_msg=speed_data

    def gear_callback(self,gear_data):
        self.gear_msg=Gear()
        self.gear_msg=gear_data

    def yaw_callback(self,yaw_data):
        self.yaw_msg=NavPVT()
        self.yaw_msg=yaw_data
        self.ctrl_pub.publish(self.ctrl_msg)

if __name__ == '__main__':
    try:
        kcity_pathtracking=erp_planner()
    except rospy.ROSInterruptException:
        pass
