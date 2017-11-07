#!/usr/bin/env python
import rospy, tf, copy, math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String

class Robot:

    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """
        self._current = Pose() # initlize correctly #Pose() 
        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        self._vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navToPose, queue_size=1) # handle nav goal events
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.readBumper, queue_size=1) # handle bumper events


    def navToPose(self,goal):
        """
        rospy.Subscriber('YOUR_STRING_HERE', ..., self.navToPose, queue_size=1) # handle nav goal events
            This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and
            then spin to match the goal orientation.
        """

        self._odom_list.waitForTransform('YOUR_STRING_HERE', 'YOUR_STRING_HERE', rospy.Time(0), rospy.Duration(1.0))
        transGoal = self._odom_list.transformPose('YOUR_STRING_HERE', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system

    def executeTrajectory(self):
        print('executeTrajectory')
        """
        See lab manual for the dance the robot has to excute
      """
        driveStraight(self, 1, .60)#go forward 60cm
        rotate(self,math.radians(90))#turn 90 right
        driveStraight(self,1,.45)#drive forward 45cm
        rotate(self,math.radians(-135))#turn 135 deg left

    def driveStraight(self, speed, distance):
        """
            This method should populate a ??? message type and publish it to ??? in order to move the robot
        """
        origin = copy.deepcopy(self._current) #hint:  use this


        q = [origin.orientation.x,
                origin.orientation.y,
                origin.orientation.z,
                origin.orientation.w] # quaternion nonsense

        xOrigin=origin.orientation.x
        yOrigin=origin.orientation.y

        atTarget=False

        move_msg=Twist()
        move_msg.linear.x=speed
        move_msg.angular.z=0

        stop_msg=Twist()
        move_msg.linear.x=0
        move_msg.linear.z=0

        currentDistance=0
        #for extra credit ramp speed from 0 to speed and from speed to 1/4 speed when past half way


        while(not atTarget and not rospy.is_shutdown()):
            if(currentDistance>=distance):
                print('driveStraight: stoped')
                atTarget=True
                self._vel_pub.publish(stop_msg)
            else:
                print('driveStraight: moving')
                origin=copy.deepcopy(self._current)
                xCurrent=origin.orientation.x
                yCurrent=origin.orientation.y
                currentDistance=math.sqrt(math.pow((xCurrent-xOrigin),2)+math.pow((yCurrent-yOrigin),2))
                self._vel_pub.publish(move_msg)
                rospy.sleep(.15)

    def spinWheels(self, v_left, v_right, time):
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

        start=rospy.Time().now().secs

        while(rospy.Time().now().secs - start <time and not rospy.is_shutdown()):
            self._vel_pub.publish(move_msg)
            print('spinwheels: moving')
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

        currentAngle=origin.orientation.w

        angle=angle+currentAngle

        if(angle==0):
            w=0
        elif(angle>0):
            w=1
        elif(angle<0):
            w=-1

        move_msg=Twist()
        move_msg.linear.x=u
        move_msg.angluar.z=w

        stop_msg =Twist()
        stop_msg.linear.x=0
        stop_msg.angular.z=0

        while(not atTarget and not rospy.is_suhtdown()):
            if(currentAngle>=angle):
                atTarget=True
                self._vel_pub.publish(stop_msg)
            else:
                origin=copy.deepcopy(self._current)
                currentAngle=origin.orientation.w
                self._vel_pub.publish(move_msg)
                rospy.sleep(.15)

   # def driveArc(self,radius,speed,angle):
        #Drive arc extra credit



    def timerCallback(self,evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
        """
        self._odom_list.waitForTransform('YOUR_STRING_HERE', 'YOUR_STRING_HERE', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('YOUR_STRING_HERE','YOUR_STRING_HERE', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
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
        if(msg.state==1):#the bumper is pressed
           executeTrajectory()

    # helper functions
    def planTraj(self, b, t):
        """
            Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
        """


if __name__ == '__main__':

    rospy.init_node('drive_base')
    turtle = Robot()

    #test function calls here
    Robot.spinWheels(turtle, 2, 5, 10)
    #Robot.driveStraight(turtle, 5, 3)

    while  not rospy.is_shutdown():
        pass
