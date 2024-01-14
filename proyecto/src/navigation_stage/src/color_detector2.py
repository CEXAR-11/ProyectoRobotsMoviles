# -*- coding: utf-8 -*-
# from __future__ import print_function
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32


UMBRAL_PIXELS = 100

class ColorDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.pub = rospy.Publisher('/color_detected', Int32, queue_size=5)
        self.publi = rospy.Publisher('/centroide_x', Int32, queue_size=5)
        self.image_sub = rospy.Subscriber('/image', Image, self.image_callback)

    def image_callback(self, msg):
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        """
        ventana_x,ventana_y = mask.shape
        print("ventana_x:",ventana_x)
        print("ventana_y:",ventana_y)
        """
        im2,contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0 :
            contour = contours[0]
            M = cv2.moments(contour)

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            cv2.circle(mask, (cx,cy), 2, (0,0,0), -1)
        cv2.imshow("Image window", mask)
        cv2.waitKey(3)

        pixels_detectados = cv2.countNonZero(mask)
        if pixels_detectados>UMBRAL_PIXELS:
            print("Detectados: ", pixels_detectados)
            self.pub.publish(pixels_detectados)
            self.publi.publish(cx)

rospy.init_node('color_detector')
cd  = ColorDetector()
rospy.spin()       