import rospy
import math
import pandas as pd
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

x, y, theta = 0.0, 0.0, 0.0
data_list = []
sensor_list = []

def newOdom(msg):
    global x, y, theta, data_list
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    q = (msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(q)
    theta = euler[2]
    data_list.append([x,y,theta])

def laser_callback(msg):
    first_18_range = []
    first_18_range = msg.ranges[0:19]
    sensor_list.append([first_18_range])

rospy.init_node("RobotController")
rospy.Subscriber("/odom", Odometry, newOdom)
rospy.Subscriber("/scan", LaserScan, laser_callback)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
speed = Twist()
r = rospy.Rate(5)

goal = [0, 2]

distance=math.sqrt(pow(goal[0]-x,2) + pow(goal[1]-y, 2))

inc_x = goal[0]-x
inc_y = goal[1]-y
angle_to_goal = math.atan2(inc_y, inc_x)
while abs(angle_to_goal - theta) > 0.1 :
    scale=1.5
    dir=(angle_to_goal-theta) /abs( angle_to_goal-theta)
    angSpeed= min(0.5, abs( angle_to_goal-theta) / scale)
    speed.angular.z = dir*angSpeed
    speed.linear.x = 0
    pub.publish(speed)
    r.sleep()


speed.angular.z = 0
while distance >= 0.7:
    distance=math.sqrt(pow(goal[0]-x,2) + pow(goal[1]-y, 2))
    rospy.loginfo(distance)
    rospy.loginfo(x)
    rospy.loginfo(y)
    speed.linear.x = 0.5
    speed.angular.z = 0
    pub.publish(speed)
    r.sleep()
speed.linear.x = 0.0
speed.angular.z =0.0
pub.publish(speed)
data_list = data_list[0:330:6]
output_df = pd.DataFrame()
output_df["ODOMETRY"] = data_list
output_df["SENSOR"] = sensor_list
output_df.to_csv('odom_data.dat')
