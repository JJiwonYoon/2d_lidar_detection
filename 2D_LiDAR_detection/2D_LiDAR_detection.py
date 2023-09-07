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
        self.global_x_LiDAR        = 240 #mm
        self.global_y_LiDAR        = 200
        self.distance              = 0   #mm
        self.point_num             = 10
        self.added_range           = int(2*((self.global_x_LiDAR)/(self.partition_cnt)))
        self.RC_group = ReentrantCallbackGroup()
        self.create_subscription(LiDAR,'/scan',self.LiDAR_callback,1, callback_group=self.RC_group)


    def LiDAR_callback(self,msg):
        self.lidar_data = msg.ranges
        self.lidar_angle_increment = msg.angle_increment
        self.partition_list        = [0,0,0,0,0,0,0,0]
        self.converted_data = []
        self.distance_data  = []
        self.x_filter       = 0
        self.y_filter       = 0
        self.partition_num  = 0
        self.target_num     = 0
        self.max_value      = 0
        for i in range(len(self.lidar_data)):
            angle = (self.lidar_angle_increment * i)
            x = -np.sin(angle) * self.lidar_data[i] * 1000
            y = np.cos(angle) * self.lidar_data[i] * 1000
            
            if (abs(x) < self.global_x_LiDAR) & (y > 0) & (y < self.global_y_LiDAR): #mm
                # partition 1
                if ((-self.global_x_LiDAR)) < (x) and (x) < ((-self.global_x_LiDAR)+(self.added_range)):
                    self.distance = (math.sqrt(x*x + y*y))
                    self.distance_data.append(self.distance)
                    self.partition_list[0] = len(self.distance_data)
                if ((-self.global_x_LiDAR)+(self.added_range)) < (x) and (x) < ((-self.global_x_LiDAR)+2*(self.added_range)):
                    self.distance = (math.sqrt(x*x + y*y))
                    self.distance_data.append(self.distance)
                    self.partition_list[1] = len(self.distance_data)
                if ((-self.global_x_LiDAR)+2*(self.added_range)) < (x) and (x) < ((-self.global_x_LiDAR)+3*(self.added_range)):
                    self.distance = (math.sqrt(x*x + y*y))
                    self.distance_data.append(self.distance)
                    self.partition_list[2] = len(self.distance_data)
                if ((-self.global_x_LiDAR)+3*(self.added_range)) < (x) and (x) < ((-self.global_x_LiDAR)+4*(self.added_range)):
                    self.distance = (math.sqrt(x*x + y*y))
                    self.distance_data.append(self.distance)
                    self.partition_list[3] = len(self.distance_data)
                if ((-self.global_x_LiDAR)+4*(self.added_range)) < (x) and (x) < ((-self.global_x_LiDAR)+5*(self.added_range)):
                    self.distance = (math.sqrt(x*x + y*y))
                    self.distance_data.append(self.distance)
                    self.partition_list[4] = len(self.distance_data)
                if ((-self.global_x_LiDAR)+5*(self.added_range)) < (x) and (x) < ((-self.global_x_LiDAR)+6*(self.added_range)):
                    self.distance = (math.sqrt(x*x + y*y))
                    self.distance_data.append(self.distance)
                    self.partition_list[5] = len(self.distance_data)
                if ((-self.global_x_LiDAR)+6*(self.added_range)) < (x) and (x) < ((-self.global_x_LiDAR)+7*(self.added_range)):
                    self.distance = (math.sqrt(x*x + y*y))
                    self.distance_data.append(self.distance)
                    self.partition_list[6] = len(self.distance_data)
                if ((-self.global_x_LiDAR)+7*(self.added_range)) < (x) and (x) < ((-self.global_x_LiDAR)+8*(self.added_range)):
                    self.distance = (math.sqrt(x*x + y*y))
                    self.distance_data.append(self.distance)
                    self.partition_list[7] = len(self.distance_data)

                self.target_num, self.max_value = max(enumerate(self.partition_list),key=lambda x: x[1])
                self.target_num +=1
        print(self.target_num,self.max_value)
                
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