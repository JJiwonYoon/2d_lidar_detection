#!/usr/bin/env python3
# -*- coding: utf-8 -*- #
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan as LiDAR
import numpy as np
import math
from std_msgs.msg import Float32
class detection(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.partition_cnt         = 8
        self.min_global_y_LiDAR    = 300
        self.global_x_LiDAR        = 500 #mm
        self.global_y_LiDAR        = 500
        self.distance              = 0   #mm
        self.added_range           = int(2*((self.global_x_LiDAR)/(self.partition_cnt)))
        self.RC_group = ReentrantCallbackGroup()
        self.create_subscription(LiDAR,'/ouster/scan',self.LiDAR_callback,1, callback_group=self.RC_group)
        self.target_num_pub = self.create_publisher(Float32, '/LiDAR_target_num', 1, callback_group=self.RC_group)


    def LiDAR_callback(self,msg):
        self.lidar_data = msg.ranges
        self.lidar_angle_increment = msg.angle_increment
        self.partition_list        = [0,0,0,0,0,0,0,0]
        self.distance_data         = []
        self.target_num            = 0
        self.max_value             = 0
        float_msg                  = Float32()
        for i in range(len(self.lidar_data)):
            angle = (self.lidar_angle_increment * i)
            x = np.sin(angle) * self.lidar_data[i] * 1000
            y = -np.cos(angle) * self.lidar_data[i] * 1000
            
            if (abs(x) < self.global_x_LiDAR) and (y > self.min_global_y_LiDAR) and (y < self.global_y_LiDAR): #mm
                for i in range(self.partition_cnt):
                    if ((-self.global_x_LiDAR)+ i*(self.added_range)) < (x) and \
                        (x) < ((-self.global_x_LiDAR)+ ((i+1)*(self.added_range))):
                        self.distance = (math.sqrt(x*x + y*y))
                        self.distance_data.append(self.distance)
                        self.partition_list[i] = len(self.distance_data)
                self.target_num, self.max_value = max(enumerate(self.partition_list),key=lambda x: x[1])
                self.target_num +=1
        float_msg.data= float(self.target_num)
        # print(f'num: {self.target_num}')
        self.target_num_pub.publish(float_msg)
                
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