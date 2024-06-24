#!/usr/bin/env python3

import time
from math import sin, cos, atan2

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Range, Imu

from hector_uav_msgs.srv import EnableMotors

import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError

from tf.transformations import euler_from_quaternion, quaternion_from_euler


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
        self.sonar_z = None
        self.imu_data = None
        self.angles_data = None
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("cam_1/camera/image", Image, self.camera_cb)
        rospy.Subscriber("cam_2/camera/image", Image, self.camera_cb2)
        rospy.Subscriber("/sonar_height", Range, self.sonar_cd)
        rospy.Subscriber("/imu", Imu, self.imu)
        self.rate = rospy.Rate(30)

        rospy.on_shutdown(self.shutdown)

    
    def take_off(self):
        start_time = time.time()
        end_time = start_time + 2
        twist_msg = Twist()
        twist_msg.linear.z = 0.5
        while (time.time() < end_time) and (not rospy.is_shutdown()):
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
    
    def sonar_cd(self, msg):
        self.sonar_z = msg.range
    
    def imu(self, msg):
        self.imu_data = msg.orientation
        self.angles_data = euler_from_quaternion ([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

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


    def geolocation2(self, Dx,Dy,h):
        fx=159.99941228826285
        xx=(Dx/h)*fx
        fy=159.99941228826285
        yy=(Dy/h)*fy
        y=xx-410
        x=308-yy
        return(x,y,xx,yy)#изображение с точкой, координаты коптера в центральной ск, координаты коптера в ск изображения
    
    def points(self, image, tang, kren, h):
        if h < 0.5: h = 0.5
        Dx = h * sin(kren)
        Dy = h * sin(tang)
        xk, yk, xi, yi = self.geolocation2(Dx, Dy, h)
        XY = []
        for i in range(2, 6):
            delta = (yi) // 6  # 5 точек делят прямую на 6 частей, потому тут стоит 6 для дельты
            obrezimage = image[int(delta * i):int(delta * i + 4), 0:820]
            BW = cv2.inRange(obrezimage, (0, 0, 0), (7, 7, 7))
            contours = cv2.findContours(BW, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours = contours[0]
            if len(contours) > 1:
                contours = sorted(contours, key=cv2.contourArea, reverse=True)
                (x, y, w, h) = cv2.boundingRect(contours[0])
                (x2, y2, w2, h2) = cv2.boundingRect(contours[1])
                matrixX = sorted([x, x + w, x2, x2 + w2])
                matrixY = sorted([y, y + h, y2, y2 + h2])
                xx = int((matrixX[1] + matrixX[2]) // 2)
                yy = int(delta * i + (matrixY[1] + matrixY[2]) // 2)
                cv2.rectangle(image, (xx, yy), (xx + 3, yy + 3), (255, 0, 0), 5)
                XY.append([(yy - 308) // (-1), (xx - 410)])
        XY.append([xk, yk])
        for i in range(1, 5):
            delta = (616 - yi) // 6
            obrezimage = image[int(delta * i + yi):int(delta * i + yi + 4), 0:820]
            BW = cv2.inRange(obrezimage, (0, 0, 0), (7, 7, 7))

            contours = cv2.findContours(BW, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours = contours[0]

            if len(contours) > 1:
                contours = sorted(contours, key=cv2.contourArea, reverse=True)
                (x, y, w, h) = cv2.boundingRect(contours[0])
                (x2, y2, w2, h2) = cv2.boundingRect(contours[1])
                matrixX = sorted([x, x + w, x2, x2 + w2])
                matrixY = sorted([y, y + h, y2, y2 + h2])
                xx = int((matrixX[1] + matrixX[2]) // 2)
                yy = int(yi + delta * i + (matrixY[1] + matrixY[2]) // 2)
                cv2.rectangle(image, (xx, yy), (xx + 3, yy + 3), (255, 0, 0), 5)
                XY.append([(yy - 308) // (-1), (xx - 410)])

        centre = [XY[1][0] - XY[4][0], XY[1][1] - XY[4][1]]
        direct = [XY[0][0] - XY[1][0], XY[0][1] - XY[1][1]]
        alfa = math.acos((centre[0] * direct[0] + centre[1] * direct[1]) / (
                    ((direct[0]) ** 2 + direct[1] ** 2) ** 0.5 * (centre[0] ** 2 + centre[1] ** 2) ** 0.5))
        return XY, centre, direct, alfa, image
    
    def RedCircle(self):
        image2 = cv2.resize(self.Image2,(600,600))
        BW=cv2.inRange(image2,(0,0,80),(10,10,255))  
        contours=cv2.findContours(BW, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours=contours[0]
        xy, pr =0, 0
        if contours:
            contours=sorted(contours, key=cv2.contourArea, reverse=True)
            cv2.drawContours(image2, contours, 0, (255,0,255), 5)
            (x,y,w,h)=cv2.boundingRect(contours[0])
            pr=(w*h/(600*600))*100
            print(pr)
            xx=(x*2+w)//2
            yy=(y*2+h)//2
            cv2.rectangle(image2,(xx,yy), (xx+3, yy+3),(255,0,0), 5)
            #cv2.imshow("555", BW)
            x=((x*2+w)//2-300)//3
            y=((y*2+h)//2-300)//(-3)
            xy=[x,y]
        return xy,pr,image2
    



    def main(self):
        self.enable_motors()
        self.take_off()
        goal_x, goal_y, goal_z = 0, 0, 1.7
        x, y, z = 0, 0, 0
        Vz, Vx, Vy = 0, 0, 0


        ##################################
        kp_z = 2; kd_z = 2
        kp_x = 0.5 * 10**(-2)
        kp_y = 6* 10**(-3)
        kd_y = 1.5
        kp_angz = 1.5

        
        time_start = time.time()
        time_now = time_start
        e_now_z = goal_z
        e_now_y = 0

        flag_start_obleta = False
        time_conets_obleta = 0
        while not rospy.is_shutdown():
            twist_msg = Twist()

            
            

            if self.enabled_gui:    
                
                x_goal, y_goal = 300, 300
                x_now, y_now = 300, 300
                y_goal_centr = 300
                x_goal_centr = 300
                centr_x, centr_y = 160, 120
                width, height = 0, 0
                xy, pr = 0, 0


                # облет колец
                if self.sonar_z is not None:
                    z = self.sonar_z
                else:
                    z - 0.1
                
                if pr > 20 and time.time() > time_conets_obleta:
                    if not flag_start_obleta: flag_start_obleta = True
                if flag_start_obleta:
                    time_conets_obleta = time.time() + 3
                    flag_start_obleta = False
                    
                if time.time() < time_conets_obleta:
                    rospy.loginfo(f"\nЛЕТИМ ВЫСОКО!")
                    goal_z = 3
                else:
                    goal_z = 1.7
                    

                if self.Image1 is not None and self.Image2 is not None:
                    
                    
                    xy,pr, image2 = self.RedCircle()

                    XY, cenrt, direct, alpha, image1 = self.points(self.Image1, self.angles_data[0], self.angles_data[1], z)
                    try:
                        x_goal, y_goal = XY[-1]
                    except:
                        pass
                    
                    cv2.circle(image1, (x_goal, y_goal), 5, (0, 0, 255), -1)
                    cv2.circle(image1, (x_now, y_now), 5, (255, 0, 0), -1)
                    try:
                        cv2.circle(image1, (XY[5][0], XY[5][1]), 5, (255, 255, 0), -1)
                        cv2.circle(image1, (XY[4][0], XY[4][1]), 5, (255, 255, 0), -1)
                        x_goal_centr = (XY[4][0]+XY[5][0])//2
                        y_goal_centr = (XY[4][1] + XY[5][1])//2
                        cv2.circle(image1, (x_goal_centr, y_goal_centr), 5, (255, 255, 255), -1)
                    except:
                        pass


                    cv2.imshow("Down view camera from Robot", image1)
                    cv2.imshow("Front view camera from Robot", image2)
                    cv2.waitKey(3)


                #вычисление ошибок
                e_now_y = x_now - x_goal_centr#cenrta_x
                ex = y_now - y_goal
                etetta = atan2(x_now - x_goal, ex)
                
                


                


                #диффиринциальная часть
                e_last_z = e_now_z 
                e_now_z = goal_z - z

                
                e_last_y = e_now_y
                e_now_y = x_now - x_goal_centr
                
                
                time_last = time_now
                time_now = time.time()
                dtime = time_now - time_last
                Vz = e_now_z * kp_z + (e_now_z - e_last_z) * dtime * kd_z
                Vy = e_now_y*kp_y + (e_now_y - e_last_y) * dtime * kd_y



                #линейный xy, угловая z
                Vx = ex*kp_x
                Vz_ang = etetta * kp_angz
                
                
                rospy.loginfo(f"e = {ex, e_now_y}\nV= {Vx, Vy, Vz},\n w = {Vz_ang}, "
                            f"z={z}\n {pr} \n {self.angles_data}")
                if time.time() - time_start >= 2:
                    #twist_msg.linear.x = Vx
                    pass
                #twist_msg.angular.z = Vz_ang
                #twist_msg.linear.y = Vy
                twist_msg.linear.z = Vz
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()

            

        

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

simple_mover = SimpleMover()
simple_mover.main()


