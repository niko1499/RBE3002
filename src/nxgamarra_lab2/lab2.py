#!/usr/bin/env python
import rospy, tf, copy, math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String
import time
import math

class Robot:

    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """
        self._current = Pose() # initlize correctly #Pose() 
        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        self._vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal2', PoseStamped, self.navToPose, queue_size=1) # handle nav goal events
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.readBumper, queue_size=1) # handle bumper events


    def navToPose(self,goal):
        """
        rospy.Subscriber('YOUR_STRING_HERE', ..., self.navToPose, queue_size=1) # handle nav goal events
            This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and
            then spin to match the goal orientation.
        """
        goalX=(goal.pose.position.x - self._current.position.x)
        goalY=(goal.pose.position.y - self._current.position.y)
        goalDistance=((goalX**2)+(goalY**2))**(.5)
        goalAngle=math.radians(math.atan2(goalY,goalX))

        self.rotate(goalAngle)
        time.sleep(2)
        self.driveStraight(1,goalDistance)

       # self._odom_list.waitForTransform('YOUR_STRING_HERE', 'YOUR_STRING_HERE', rospy.Time(0), rospy.Duration(1.0))
       # transGoal = self._odom_list.transformPose('YOUR_STRING_HERE', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system

    def driveStraight(self, speed, distance):
        """
            This method should populate a ??? message type and publish it to ??? in order to move the robot
        """
        origin = copy.deepcopy(self._current) #hint:  use this


        q = [origin.orientation.x,
                origin.orientation.y,
                origin.orientation.z,
                origin.orientation.w] # quaternion nonsense

        xOrigin=self._current.position.x
        yOrigin=self._current.position.y
        atTarget=False

        move_msg=Twist()
        move_msg.linear.x=speed
        move_msg.angular.z=0

        stop_msg=Twist()
        stop_msg.linear.x=0
        stop_msg.linear.z=0

        currentDistance=0
        #for extra credit ramp speed from 0 to speed and from speed to 1/4 speed when past half way
        vel=0

        while(not atTarget and not rospy.is_shutdown()):
            if(currentDistance>=distance):
                print('driveStraight: stoped')
                atTarget=True
                self._vel_pub.publish(stop_msg)
            else:
                print('driveStraight: moving')
                origin=copy.deepcopy(self._current)
                xCurrent=self._current.position.x
                yCurrent=self._current.position.y
                currentDistance=math.sqrt(math.pow((xCurrent-xOrigin),2)+math.pow((yCurrent-yOrigin),2))
                self._vel_pub.publish(move_msg)
                print('current x: '+str(xCurrent)+'current y: '+str(yCurrent)+'origin x: '+str(xOrigin)+'origin y:'+str(yOrigin))
                print('\n distance: '+str(currentDistance))
               # rospy.sleep(.15)

    def spinWheels(self, v_left, v_right, duration):
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
           It should then create a ??? message type, and publish it to ??? in order to move the robot
        """
        print('spinwheels')
        diameter = 0.23 # based on wheel track from https://yujinrobot.github.io/kobuki/doxygen/enAppendixKobukiParameters.html


        driveStartTime = rospy.Time.now().secs

        w=(v_right-v_left)/diameter

        u=(v_left+v_right)/2

        move_msg=Twist()
        move_msg.linear.x=u
        move_msg.angular.z=w

        stop_msg =Twist()
        stop_msg.linear.x=0
        stop_msg.angular.z=0

        start=time.time()
        currentTime=start

        while(currentTime - start <duration and not rospy.is_shutdown()):
            currentTime=time.time()
            self._vel_pub.publish(move_msg)
            print('spinwheels: moving')
            print('\n time: '+str(time)+'start: '+str(start)+'current: '+str(currentTime))
        self._vel_pub.publish(stop_msg)
        print('spinwheels: stoped')


    def rotate(self,angle):
        """
            This method should populate a ??? message type and publish it to ??? in order to spin the robot
        """
        origin = copy.deepcopy(self._current)

        q = [origin.orientation.x,
                origin.orientation.y,
                origin.orientation.z,
                origin.orientation.w] # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)

        atTarget=False

        currentAngle=yaw
        angle=angle+currentAngle

        if(angle==currentAngle):
            w=0
        elif(angle>currentAngle):
            w=1
        elif(angle<currentAngle):
            w=-1

        move_msg=Twist()
        move_msg.linear.x=0
        move_msg.angular.z=w


        stop_msg =Twist()
        stop_msg.linear.x=0
        stop_msg.angular.z=0

        while(not atTarget and not rospy.is_shutdown()):
            if(currentAngle>=angle):
                atTarget=True
                self._vel_pub.publish(stop_msg)
                print('rotate: stoped')
            else:
                origin = copy.deepcopy(self._current)

                q = [origin.orientation.x,
                        origin.orientation.y,
                        origin.orientation.z,
                        origin.orientation.w] # quaternion nonsense

                (roll, pitch, yaw) = euler_from_quaternion(q)

                currentAngle=yaw
                self._vel_pub.publish(move_msg)
                rospy.sleep(.15)
                print('rotate: moving')
                print('angle: '+str(angle)+'currentAngle: '+str(currentAngle))

   # def driveArc(self,radius,speed,angle):
        #Drive arc extra credit



    def timerCallback(self,evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
        """
        self._odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
        self._current.position.x = position[0]
        self._current.position.y = position[1]

        self._current.orientation.x = orientation[0]
        self._current.orientation.y = orientation[1]
        self._current.orientation.z = orientation[2]
        self._current.orientation.w = orientation[3]
        q = [self._current.orientation.x,
                self._current.orientation.y,
                self._current.orientation.z,
                self._current.orientation.w] # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)


    def readBumper(self, msg):
        #
        """
        callback function that excutes on a BumperEvent
        """
       # if(msg.state==1):#the bumper is pressed
        self.executeTrajectory()

    # helper functions
    def planTraj(self, b, t):
        """
            Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
        """


    def executeTrajectory(self):
        print('executeTrajectory')
        """
        See lab manual for the dance the robot has to excute
      """
        self.driveStraight(1,.60)#go forward 60cm
        self.rotate(math.radians(90))#turn 90 right
        self.driveStraight(1,.45)#drive forward 45cm
        self.rotate(math.radians(-135))#turn 135 deg left



if __name__ == '__main__':

    rospy.init_node('drive_base')
    turtle = Robot()
    rospy.sleep(2)
    print 'ready'

    #test function calls here
    #Robot.spinWheels(turtle, 2, 5, 3)
   # Robot.rotate(turtle,math.radians(90))
    Robot.driveStraight(turtle, 5, 1)
    #Robot.rotate(turtle,math.radians(90))
    #Robot.executeTrajectory(turtle)

    while  not rospy.is_shutdown():
        pass
