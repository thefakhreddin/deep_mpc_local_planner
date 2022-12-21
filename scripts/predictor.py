#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist

import tensorflow as tf
from tensorflow import keras
from keras.models import load_model
import os
import numpy as np
from skimage.transform import downscale_local_mean



deepMPC = load_model(
    os.path.join(
        os.path.dirname(__file__),
        '../model/deepMPCModel'))

heading = np.array([0,0])
cmdVel = np.array([0,0])
currentPose = np.array([0,0])
costMap = np.zeros((20,20))

def costMapCallback(data):
    highRes = np.array(data.data)
    highRes = np.reshape(highRes,(60,60))
    global costMap
    costMap = downscale_local_mean(highRes, (3,3))
    

def headingStorageCallback(data):
    global heading
    global currentPose
    heading = np.array([data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    currentPose = np.array([data.pose.pose.position.x, data.pose.pose.position.y])

def predictionCallback(data):
    planX = np.array([d.pose.position.x - currentPose[0] for d in data.poses[:50]])
    planY = np.array([d.pose.position.y - currentPose[1] for d in data.poses[:50]])
    plan = np.stack([planX, planY], axis=1)
    global cmdVel
    try:
        cmdVel = deepMPC.predict([plan[None,:], heading[None,:], costMap[None,:]]).reshape((-1))
    except:
        pass

def listener():
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, predictionCallback)
    rospy.Subscriber("/odom", Odometry, headingStorageCallback)
    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, costMapCallback)

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cmdVelObject = Twist()
        global cmdVel
        cmdVelObject.linear.x = cmdVel[0]
        cmdVelObject.angular.z = cmdVel[1]
        pub.publish(cmdVelObject)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('deepMPCModel', anonymous=True)
    try:
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass











