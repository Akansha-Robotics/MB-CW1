#!/usr/bin/env python

#Import rospy for creating nodes in ros
import rospy

# Import the message type which is required for Publisher
# Need to also add dependencies in package.xml
from geometry_msgs.msg import Twist

# Import the message type which is required for Subscriber 
from turtlesim.msg import Pose

# IMP NOTE: To check message type: 
# rostopic info /turtle1/cmd_vel - for controlling the robot
# rostopic info /turtle1/pose - for knowing position of robot 

# To check what the message is: rosmsg show geometry_msgs/Twist
# rosmsg show geometry_msgs/Pose

# Setting limits of window so we know where the wall is 
# Setting safe to be 2 so that the robot know that it will crash before hand 
# and can take action to avoid
x_max_limit = 11.0
y_max_limit = 11.0
x_min_limit = 0.0
y_min_limit = 0.0
safe = 2.0
# Through trial and error, I found 2 to be a good safe distance allowing enough space for robot to turn 

# robot current position is entered in varaible called robot_pose
robot_pose = Pose()
# If you do not set the above line, the program will not know the location of robot 
# and will give the error: NameError: name 'robot_pose' is not defined

# below function is used to update robot position 
def callback(pose):
    # global robot_pose would ensure that we maintaining the current variable 
    # and not creating new ones 
    global robot_pose
    robot_pose = pose

def crash_checker():
    # Function checks whether robot is going to crash wall 
    
    # Calculated by checking if 
    # Robot position is lesser than minimum value limits plus the safe distance - (1.0)
    # Robot position is greater than the maximum values minus than the safe distance - (10.0)

    near_crash = (robot_pose.x < x_min_limit + safe or
                 robot_pose.x > x_max_limit - safe or
                 robot_pose.y < y_min_limit + safe or
                 robot_pose.y > y_max_limit - safe)
    # The output of near_crash is True or False (Boolean) 
    # OR logical operator is used so that if any of the condition is True and robot is near crash
    # near_crash will be true 


    # if loop below simply prints a warning message if near_crash is True 
    # and then constantly provides the value of near_crash through return command 
    if near_crash:
        rospy.loginfo("Robot near wall")

    return near_crash

# Main function for controlling robot 
def robot_move():
    
    # setting pub to be global variable
    # allows this variable to be used in different scripts 
    # (useful when you are using multiple scripts and need to access variable) 
    global pub

    # setting vel_msg to be Twist message type as that is required to control robot through cmd_vel
    vel_msg = Twist()

    # Initialize a ROS node - 'crash_avoid'. 
    # 'anonymous=True' ensures that if multiple instances of this node are launched,
    # they won't clash with each other (they'll have unique names).
    rospy.init_node('crash_avoid', anonymous=True)
    
    # creates a publisher that will post control commands to cmd_vel to move robot 
    # Twist is specified as that is the message type for cmd_vel 
    # queue_size=10 - it can store up to 10 messages before old ones are discarded.
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # creates a subscribe that will receive info from pose to know location of robot 
    # Pose is specified as that is the message type for it 
    # setting the function callback ensures message is called when published 
    rospy.Subscriber('/turtle1/pose', Pose, callback)

    # processes data at rate of 10 hertz (matches queue_size)
    rate = rospy.Rate(10)

# function states that while robot is running 
# if crash_checker is True - it will turn 1 step and go forward 1 step 
# otherwise it will keep going forward 2 steps 
    while not rospy.is_shutdown():
        if crash_checker():
            vel_msg.linear.x = 1.0
            vel_msg.angular.z = 1.0
        else:
            vel_msg.linear.x = 2.0
            vel_msg.angular.z = 0.0

# will publish commands from vel_msg to pub which is the variable used to control robot
        pub.publish(vel_msg)
# will tell program to calm down
        rate.sleep()

# below code ensure that robot_move function is running when scripted is runned 
# function will stop running when it is stopped by user 
if __name__ == '__main__':
    try:
        robot_move()
    except rospy.ROSInterruptException:
        pass
