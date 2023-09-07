#!/usr/bin/env python3
# -*- coding: utf-8 -*- #
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan as LiDAR
import numpy as np
import math
from visualization_msgs.msg import Marker

class detection(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.filter_distance       = 0 ##mm
        self.partition_cnt         = 8
        self.global_x_LiDAR        = 200 #mm
        self.global_y_LiDAR        = 200
        self.distance              = 0   #mm
        self.point_num             = 1
        
        self.RC_group = ReentrantCallbackGroup()
        self.create_subscription(LiDAR,'/scan',self.LiDAR_callback,1, callback_group=self.RC_group)


    def LiDAR_callback(self,msg):
        self.lidar_data = msg.ranges
        self.lidar_angle_increment = msg.angle_increment
        self.converted_data = []
        self.distance_data  = []
        self.x_filter       = 0
        self.y_filter       = 0
        self.partition_num  = 0

        for i in range(len(self.lidar_data)):
            angle = (self.lidar_angle_increment * i)
            x = -np.sin(angle) * self.lidar_data[i] * 1000
            y = np.cos(angle) * self.lidar_data[i] * 1000
            if (abs(x) < self.global_x_LiDAR) & (y > 0) & (y < self.global_y_LiDAR): #mm
                
                self.distance = (math.sqrt(x*x + y*y))
                self.distance_data.append(self.distance)
                if len(self.distance_data) > (self.point_num):
                    self.converted_data.append([x,y])
                    x_values = [item[0] for item in self.converted_data]
                    y_values = [item[1] for item in self.converted_data]
                    self.x_filter = sum(x_values) / len(x_values)
                    self.y_filter = sum(y_values) / len(y_values)
                    self.filter_distance = (math.sqrt((self.x_filter*self.x_filter)+(self.y_filter*self.y_filter)))
                    # print("x_filter:", self.x_filter,"y_filter:", self.y_filter)
                    # print(((-self.global_x_LiDAR/2)+((self.global_x_LiDAR)/(self.partition_cnt))))
                    # print(self.filter_distance)
                    if ((-self.global_x_LiDAR)/2) < (self.x_filter) and (self.x_filter) < ((-self.global_x_LiDAR/2)+((self.global_x_LiDAR)/(self.partition_cnt))):
                        if self.filter_distance > 300:
                            self.partition_num = 1
                            print(self.partition_num)
                            
                        

def main():
        rclpy.init()
        C = detection()
        try:
            rclpy.spin(C)
        finally:
            C.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()