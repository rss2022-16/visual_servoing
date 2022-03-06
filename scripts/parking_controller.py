#!/usr/bin/env python2

import math
import rospy
import numpy as np
import purepursuit as pp

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = 0.75 # meters; try playing with this number!
        self.angle_tolerance = 0.05 
        #calculate acceptable angle error proportion to parking_distance
        self.parking_angle = abs(math.atan(self.angle_tolerance/self.parking_distance))
        #acceptable error regarding parked distance away
        self.park_tolerance = 0.15

        #cone coordinate
        self.relative_x = 0
        self.relative_y = 0
        self.velocity = 0.5
        

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos

        #purepursuit arguments
        look_ahead = 0.3
        car_length = 0.37

        v = None
        steer = None
        distance = self.check_distance()
        angle = self.check_angle()    
        print(distance)
        print(angle)

        #calculate the turning angle needed to park in front of cone
        if(abs(distance - self.parking_distance) < self.park_tolerance and abs(angle) < self.parking_angle):
            v = 0.0
            steer = 0.0
        # elif(abs(angle) > self.parking_angle):
        #     if(distance < self.parking_distance + self.park_tolerance):
        #         v = -self.velocity
        #         steer = -(math.atan(self.relative_y/self.relative_x))
        #     else:
        #         (steer, v) = pp.purepursuit(look_ahead, car_length, self.velocity, 0, 0, 0, np.array(([0,0], [self.relative_x, self.relative_y])))
        else:
            if(abs(angle) > self.parking_angle):
                if(distance < self.parking_distance + 0.5):
                    v = -self.velocity
                    steer = -(math.atan(self.relative_y/self.relative_x))
                else:
                    (steer, v) = pp.purepursuit(look_ahead, car_length, self.velocity, 0, 0, 0, np.array(([0,0], [self.relative_x, self.relative_y])))
            else:
                if(distance < self.parking_distance):
                    v = -self.velocity
                    steer = -(math.atan(self.relative_y/self.relative_x))
                else:
                    (steer, v) = pp.purepursuit(look_ahead, car_length, self.velocity, 0, 0, 0, np.array(([0,0], [self.relative_x, self.relative_y])))

        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.header.frame_id = "base_link"
        drive_cmd.drive.speed = v
        drive_cmd.drive.steering_angle = steer
        
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    # find the distance between target coordinate and car
    def check_distance(self):
        return math.sqrt(self.relative_x**2 + self.relative_y**2)

    # find the angle the car is facing away from target coordinate
    def check_angle(self):
        return math.atan(self.relative_y/self.relative_x)

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = self.check_distance()
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
