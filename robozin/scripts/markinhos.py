from serial_com import SerialControllerInterface
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np
import math
import rospy


class Markinhos:
    def __init__(self):
        self.com = SerialControllerInterface()
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.vel = Twist()

        self.GOT_START_ANGLE = False
        self.start_angle = 0
        self.angle = 0
        self.front = 0
        self.seted_front = 0
        self.right = 0
        self.seted_right = 0
        self.back = 0
        self.seted_back = 0
        self.left = 0
        self.seted_left = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.spd_multiplier = 0.25
        self.max_speed = 1

    def lidar_cb(self, dado):
        self.front = dado.ranges[0]
        self.right = dado.ranges[90]
        self.back = dado.ranges[180]
        self.left = dado.ranges[270]
        if self.com.get_connected():
            if self.front <= 0.5 and self.front > 0.01 and self.seted_front == 0:
                self.com.send_lidar_data(b"N", 1)
                self.seted_front = 1
            elif (self.front > 0.5 or math.isinf(self.front)) and self.seted_front == 1:
                self.com.send_lidar_data(b"N", 0)
                self.seted_front = 0
            if self.right <= 0.5 and self.right > 0.01 and self.seted_right == 0:
                self.com.send_lidar_data(b"L", 1)
                self.seted_right = 1
                print("sended L 1")
            elif (self.right > 0.5 or math.isinf(self.right)) and self.seted_right == 1:
                self.com.send_lidar_data(b"L", 0)
                self.seted_right = 0
                print("sended L 0")
            if self.back <= 0.5 and self.back > 0.01 and self.seted_back == 0:
                self.com.send_lidar_data(b"S", 1)
                print("sended S 1")
                self.seted_back = 1
            elif (self.back > 0.5 or math.isinf(self.back)) and self.seted_back == 1:
                self.com.send_lidar_data(b"S", 0)
                print("sended S 0")
                self.seted_back = 0
            if self.left <= 0.5 and self.left > 0.01 and self.seted_left == 0:
                self.com.send_lidar_data(b"O", 1)
                self.seted_left = 1
                print("sended O 1")
            elif (self.left > 0.5 or math.isinf(self.left)) and self.seted_left == 1:
                self.com.send_lidar_data(b"O", 0)
                self.seted_left = 0
                print("sended O 0")

    def odom_cb(self, dado):
        quat = dado.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]
        angulos = np.degrees(transformations.euler_from_quaternion(lista))
        if angulos[2] < 0:
            self.angle = 360 + angulos[2]
        else:
            self.angle = angulos[2]

    def set_speed(self):
        self.vel.linear.x = self.vel_x * self.spd_multiplier
        self.vel.linear.y = self.vel_y * self.spd_multiplier
        self.vel.angular.z = self.vel_z * self.spd_multiplier
        self.vel_pub.publish(self.vel)

    def update(self):
        if not self.com.get_connected():
            self.vel_x = 0
            self.vel_y = 0
            self.vel_z = 0
            self.set_speed()
            try:
                self.com.connecting()
            except Exception as e:
                print(e)
        else:
            self.com.get_com_data()

            if self.com.get_turn_around():
                if not self.GOT_START_ANGLE:
                    self.GOT_START_ANGLE = True
                    self.start_angle = self.angle
                print(f"180? {abs((360 + self.start_angle - self.angle) % 360 - 180) > 5}")
                if abs((360 + self.start_angle - self.angle) % 360 - 180) > 5:
                    self.vel_x = 0
                    self.vel_z = 1
                    self.set_speed()
                else:
                    self.vel_x = 0
                    self.vel_z = 0
                    self.set_speed()
                    self.com.set_turn_around(False)
                    self.GOT_START_ANGLE = False
            else:
                if self.com.SPD_UP and self.spd_multiplier <= 1:
                    self.spd_multiplier += 0.1
                    self.com.set_spd_up(False)
                elif self.com.SPD_DOWN and self.spd_multiplier >= 0.25:
                    self.spd_multiplier -= 0.1
                    self.com.set_spd_down(False)

                if self.com.analog_val[b"X"] > 1900 and self.com.analog_val[b"X"] < 2900:
                    self.vel_x = 0
                elif self.com.analog_val[b"X"] > 3000 and self.com.analog_val[b"X"] < 4000:
                    self.vel_x = self.max_speed / 2
                elif self.com.analog_val[b"X"] >= 4000:
                    self.vel_x = self.max_speed
                elif self.com.analog_val[b"X"] > 100 and self.com.analog_val[b"X"] < 1800:
                    self.vel_x = -self.max_speed / 2
                elif self.com.analog_val[b"X"] <= 100:
                    self.vel_x = -self.max_speed

                if self.com.analog_val[b"Y"] > 1900 and self.com.analog_val[b"Y"] < 2900:
                    self.vel_z = 0
                elif self.com.analog_val[b"Y"] > 3000 and self.com.analog_val[b"Y"] < 4000:
                    self.vel_z = -(self.max_speed + 1) / 2
                elif self.com.analog_val[b"Y"] >= 4000:
                    self.vel_z = -(self.max_speed + 0.5)
                elif self.com.analog_val[b"Y"] > 100 and self.com.analog_val[b"Y"] < 1800:
                    self.vel_z = (self.max_speed + 1) / 2
                elif self.com.analog_val[b"Y"] <= 100:
                    self.vel_z = (self.max_speed + 0.5)

                
                self.set_speed()
