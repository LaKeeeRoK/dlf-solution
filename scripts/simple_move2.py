#!/usr/bin/env python3

import time
from math import sin, cos

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
#from geometry_msgs.msg import PoseSteamped

from hector_uav_msgs.srv import EnableMotors

import cv2
from cv_bridge import CvBridge, CvBridgeError

class SimpleMover():

    def __init__(self):
        rospy.init_node('simple_mover', anonymous=True)

        if rospy.has_param('gui'):
            self.enabled_gui = rospy.get_param('gui')
        else:
            rospy.logerr("Failed to get param 'gui'")

        self.cv_bridge = CvBridge()
        self.Image1 = None
        self.Image2 = None
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("cam_1/camera/image", Image, self.camera_cb)
        rospy.Subscriber("cam_2/camera/image", Image, self.camera_cb2)
        #rospy.Subscriber("/pose", PoseSteamped, self.position)
        self.rate = rospy.Rate(30)

        #
        # cv2.namedWindow("uraaa")
        # cv2.namedWindow("uraaa 2")
        # rospy.logerr("named window created")



        rospy.on_shutdown(self.shutdown)
    
    def position(self, msg):
        pass
                


    def camera_cb(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.Image1 = cv_image
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def camera_cb2(self, msg):
        try:
            cv_image2 = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.Image2 = cv_image2
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def enable_motors(self):

        try:
            rospy.wait_for_service('enable_motors', 2)
            call_service = rospy.ServiceProxy('enable_motors', EnableMotors)
            response = call_service(True)
        except Exception as e:
            print("Error while try to enable motors: ")
            print(e)


    def take_off(self):

        self.enable_motors()

        start_time = time.time()
        end_time = start_time + 3
        twist_msg = Twist()
        twist_msg.linear.z = 1.0

        while (time.time() < end_time) and (not rospy.is_shutdown()):
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()



    def spin(self):

        self.take_off()

        start_time = time.time()
        while not rospy.is_shutdown():
            twist_msg = Twist()
            t = time.time() - start_time
            twist_msg.linear.z = 0.8 * cos(1.2 * t)
            twist_msg.linear.y = 0.8 * sin(0.6 * t)
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
            if self.enabled_gui:
                if self.Image1 is not None and self.Image2 is not None:
                    cv2.imshow("Down view camera from Robot", self.Image1)
                    cv2.imshow("Front view camera from Robot", self.Image2)
                    cv2.waitKey(3)

    def check_moving(self):
        self.take_off()
        rospy.loginfo(f"Поднялись в верх") 
        time_now= time.time()
        time_start = time.time()
        goal_x, goal_y, goal_z =  0, 0, 0
        p = 0.01
        x, y, z, phi, th, fi = [0, 0, 0, 0, 0, 0]
        Vx, Vy, Vz = 0, 0, 0
        rospy.loginfo(f"Поднялись в верх") 
        dtime = 0.1
        while not rospy.is_shutdown():
            twist_msg = Twist()
            '''t = time.time() - start_time
            twist_msg.linear.z = 0.8 * cos(1.2 * t)
            twist_msg.linear.y = 0.8 * sin(0.6 * t)
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()'''
            
            # if time_now - time_last < dtime:
            #     time.sleep(dtime - time_now + time_last)
            #     rospy.loginfo(f"спали {dtime - time_now + time_last}") 
            # else:
            #     rospy.loginfo("!!!!!")
            # time_now = time.time()

                #Рабочий код
            Vx_last = Vy
            Vy_last = Vy
            Vz_last = Vz
           

            Vx = p * (goal_x - x)
            Vy = p * (goal_y - y)
            Vz = p * (goal_z - z)
            # if Vz < 0:
            #     twist_msg.linear.z = Vz * 0.01
            #     rospy.loginfo(f"ААААААААААААААААААААААА\n\n\n\n") 
            # else:
            #     twist_msg.linear.z = Vz
            twist_msg.linear.z = Vz
            twist_msg.linear.x = 0
            twist_msg.linear.y = 0
            rospy.loginfo(Vx, Vy, Vz) 
            time_last = time_now
            time_now = time.time()
            dtime = (time_now - time_last)
            z += (Vz_last + Vz)/2 * dtime
            x += (Vx_last + Vx)/2 * dtime
            y += (Vy_last + Vy)/2 * dtime
            rospy.loginfo(f"Vz = {Vz}; z = {z}")
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
                
    


            if self.enabled_gui:
                if self.Image1 is not None and self.Image2 is not None:
                    cv2.imshow("Down view camera from Robot", self.Image1)
                    cv2.imshow("Front view camera from Robot", self.Image2)
                    cv2.waitKey(3)
                    rospy.loginfo(f"img1:{cv2.ds}")
    def start(self):
        self.enable_motors()
        time_start = time.time()
        while not rospy.is_shutdown():
            twist_msg = Twist()
            z = 0
            if time.time() - time_start <= 1:
                z = 1
            twist_msg.linear.z = z
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
            rospy.loginfo(f"Линейнская скорость = {z}, прошло  {time.time() - time_start}")



    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


simple_mover = SimpleMover()


simple_mover.start()