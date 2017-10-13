#!/usr/bin/env python

# TODO: add feedforward velocity to pubVeloc

import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import AccelStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8

class Demo():
    def __init__(self, goals):
        rospy.init_node('demo', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.pubAccel = rospy.Publisher('feedforwardAcceleration', AccelStamped, queue_size=1)
        self.pubVeloc = rospy.Publisher('feedforwardVelocity', TwistStamped, queue_size = 1)
        self.pubCircle = rospy.Publisher('docircle', Int8, queue_size = 1)
        self.listener = TransformListener()
        self.goals = goals
        self.goalIndex = 0

    def run(self):
        self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.worldFrame
        circleFlag = 0
        acc = AccelStamped()
        acc.header.seq = 0
        acc.header.frame_id = self.worldFrame
        vel = TwistStamped()
        vel.header.seq = 0
        vel.header.frame_id = self.worldFrame
        doCircle = Int8()
        doCircle.data = 0


        # rate = rospy.Rate(200)  # control the publish rate?

        while not rospy.is_shutdown():
            # hovering at the first given position
            goal.header.seq += 1
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = self.goals[self.goalIndex][0]
            goal.pose.position.y = self.goals[self.goalIndex][1]
            goal.pose.position.z = self.goals[self.goalIndex][2]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.goalIndex][3])
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]

            acc.header.seq += 1
            acc.header.stamp = rospy.Time.now()
            acc.accel.linear.z = 0

            vel.header.seq += 1
            vel.header.stamp = rospy.Time.now()
            vel.twist.linear.x = 0
            vel.twist.linear.y = 0

            self.pubGoal.publish(goal)
            self.pubAccel.publish(acc)
            self.pubVeloc.publish(vel)
            self.pubCircle.publish(doCircle)
            # rate.sleep()

            t = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
            if self.listener.canTransform(self.worldFrame, self.frame, t):
                position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame, t)
                rpy = tf.transformations.euler_from_quaternion(quaternion)
                # if     math.fabs(position[0] - self.goals[self.goalIndex][0]) < 0.1 \
                #    and math.fabs(position[1] - self.goals[self.goalIndex][1]) < 0.1 \
                #    and math.fabs(position[2] - self.goals[self.goalIndex][2]) < 0.1 \
                #    and math.fabs(rpy[2] - self.goals[self.goalIndex][3]) < math.radians(5):
                if     math.fabs(position[0] - self.goals[self.goalIndex][0]) < 0.1 \
                   and math.fabs(position[1] - self.goals[self.goalIndex][1]) < 0.1 \
                   and math.fabs(position[2] - self.goals[self.goalIndex][2]) < 0.1 \
                   and math.fabs(rpy[2] - 0) < math.radians(5):
                        rospy.sleep(5)
                        circleFlag += 1
                        doCircle.data = 1


            t0 = rospy.get_time()


            while circleFlag == 1:
                goal.header.seq += 1
                goal.header.stamp = rospy.Time.now()
                t = rospy.get_time() - t0
                # n = 0.5
                # if t > 10 and t <= 20:
                #     n = 1
                # else:
                #     n = 2
                # change the rate
                if t > 32:
                    circleFlag = 2
                    docircle = 0
                # continuously generate the setpoint
                goal.pose.position.x = 0.5*math.cos(math.radians(45*2*t))
                goal.pose.position.y = 0.5*math.sin(math.radians(45*2*t)) - 0.5
                goal.pose.position.z = self.goals[self.goalIndex][2]

                x_dt = -0.5*math.radians(45*2)*math.sin(math.radians(45*2*t))
                y_dt = 0.5*math.radians(45*2)*math.cos(math.radians(45*2*t))

                # 
                x_ddt = -0.5*math.radians(45*2)*math.radians(45*2)*math.cos(math.radians(45*2*t))
                y_ddt = -0.5*math.radians(45*2)*math.radians(45*2)*math.sin(math.radians(45*2*t))

                # publish x_acc and y_acc
                acc.header.seq += 1
                acc.header.stamp = rospy.Time.now()
                # add scaling factor
                acc.accel.linear.x = x_ddt * 180 / math.pi / 9.8
                acc.accel.linear.y = y_ddt * 180 / math.pi / 9.8
                acc.accel.linear.z = 1
                z_ddt = 0

                # publish x_vel and y_vel
                vel.header.seq += 1
                vel.header.stamp = rospy.Time.now()
                vel.twist.linear.x = x_dt
                vel.twist.linear.y = y_dt

                # m = 0.023

                ########################differential flatness##########################
                # g = 9.8

                # fzb = np.array([[x_ddt,y_ddt,z_ddt+g]])
                # zb = fzb/np.linalg.norm(fzb)

                # zbx = zb[0][0]
                # zby = zb[0][1]
                # zbz = zb[0][2]
                
                # #roll
                # roll = math.atan2(zbz,zby)-math.pi/2
                # #pitch
                # pitch = math.atan2(zbx,zbz)
                #######################################################################

                quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)

                # acc.accel.angular.x = roll
                # acc.accel.angular.y = pitch

                goal.pose.orientation.x = quaternion[0]
                goal.pose.orientation.y = quaternion[1]
                goal.pose.orientation.z = quaternion[2]
                goal.pose.orientation.w = quaternion[3]

                self.pubGoal.publish(goal)
                self.pubAccel.publish(acc)
                self.pubVeloc.publish(vel)
                self.pubCircle.publish(doCircle)
                # rate.sleep()

                # t = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
                # if self.listener.canTransform(self.worldFrame, self.frame, t):
                #     position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame, t)
                #     rpy = tf.transformations.euler_from_quaternion(quaternion)
                #     # if     math.fabs(position[0] - goal.pose.position.x) < 0.2 \
                #     #    and math.fabs(position[1] - goal.pose.position.y) < 0.2 \
                #     #    and math.fabs(position[2] - goal.pose.position.z) < 0.1 \
                #     #    and math.fabs(rpy[2] - goal.pose.orientation.w) < math.radians(10):
                #     if     math.fabs(position[0] - goal.pose.position.x) < 0.2 \
                #        and math.fabs(position[1] - goal.pose.position.y) < 0.2 \
                #        and math.fabs(position[2] - goal.pose.position.z) < 0.1 \
                #        and math.fabs(rpy[2] - 0) < math.radians(10):
                #        continue
