#!/usr/bin/env python

# Import rospy for creating nodes in ros
import rospy

# Import the message type which is required for Publisher to know location of robot
from geometry_msgs.msg import Twist

# Import the message type which is required for Subscriber
from turtlesim.msg import Pose

# Import for conducting mathematical operations
# Was used for calculating distance to move between current and goal point 
import math

# Import to aid in flow of code by allowing and blocking certain operations
# Was used to hold taking 2nd user values input until 1st position values was achieved 
import threading

# Initializing Pose in current and goal point variables 
# If you do not initialize Pose(), it will give NameError that variable is not defined 
current_point = Pose()
goal_point = Pose()

# Creating X and Y variables for goal point which will be set by user input 
goal_point.x = None
goal_point.y = None


# Makes goal_reached variable an Event from Threading Module which communicates when output is True or False
# Will be used to communicate when robot has reached goal to ask user for 2nd goal point 
goal_reached = threading.Event()

# Communicate current action of robot
# Default is False as robot is standing still turtlesim_node is runned 
robot_moving = False

# Allow user to enter the goal point without waiting 
# Marked True as when first open turtlesim_node, it will automatically be first input 
# If below variable is not added, the program will face an error in which user can not enter goal point without the goal_reached event being marked True
first_input = True

# Simple function to update current location of robot into current_point variable 
# will be used when creating Subscriber 
def callback(pose):
    global current_point
    current_point = pose


# Function is for moving robot towards the goal point set 
def to_goal():

    # Setting global variables that will be used in function 
    global current_point, goal_point, robot_moving, first_input

    # Initializing vel_msg variable with Twist() as that is message type of cmd_vel
    vel_msg = Twist()
    # processes data at rate of 10 hertz
    rate = rospy.Rate(10)  # 10 Hz

    # While loop continues to work as long as ROS node is running 
    while not rospy.is_shutdown():

    	# If loop states that if no value is in either X or Y of goal_point
        # robot will sleep or wait for goal 
        if goal_point.x is None or goal_point.y is None:
            rate.sleep()
            continue

        # Calculate the distance and angle to the goal

        # Finding Euclidean Distance (straight line lenght) using Pythagorean Theorem 
        # First, differences squared and then summed up = (x2​−x1​)^2+(y2​−y1​)^2
        # Then we square root this using math.sqrt 
        distance = math.sqrt((goal_point.x - current_point.x) ** 2 + (goal_point.y - current_point.y) ** 2)
        
        
        # Finding angle between goal and current point - Used for positioning which direction robot should face 
        # First, calculates differences = y2-y1 , x2-x1
        # Then using math.atan2, we find angle in radians
        angle_to_goal = math.atan2(goal_point.y - current_point.y, goal_point.x - current_point.x)

        # Below if loop will giving instructions on moving robot 
        # If the distance calculated is greater than 0.1, then controls will be published to cmd_vel and robot will move toward goal 
        # Else if distance is less than 0.1 (which 0), then robot will not move and variables will be marked accordingly 
        if distance > 0.1:

            # When distance is greater than 0.1, robot will be moving to point so robot_moving is set True 
            robot_moving = True

            # Setting linear velocity to move with speed ranging from 0.5 to 1.5 times the distance 
            # Ensure robot will not move too fast if distance is far 
            vel_msg.linear.x = min(1.5 * distance, 0.5)  # Caps the linear speed
            
            # Setting angular velocity (for robot rotation) 
            # Based on difference from goal angle (Value calculated above) from current angle (Theta)
            vel_msg.angular.z = 4 * (angle_to_goal - current_point.theta)

            # Has proportional control according to distance and angle 
            # Values were determined through simple trial and error 
        else:

            # If the distance is less than 0.1
            # that equals to 0 and robot is at goal point so robot_moving is set to False 
            robot_moving = False

            # Linear and angular are marked to 0 to stop robot from moving 
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0

            # Control message for cmd_vel gets published and successful message gets printed on terminal 
            pub.publish(vel_msg)
            rospy.loginfo("Robot has reached point")

            # goal_reached threading event is set along with first_input set to False 
            # to indicate that program needs to wait until goal is marked reached for taking next user input 
            goal_reached.set()  
            first_input = False 

            # Clear values in goal point to allow next values to be filled 
            goal_point.x = None
            goal_point.y = None


        # If robot_moving is True, then vel_msg will publish commands to move robot 
        #  rate.sleep() used to regulate speed of loop and not overpressure robot by going too fast
        if robot_moving:
            pub.publish(vel_msg)

        rate.sleep()

    # Simply stop robot after goal reached and publish this stop command to cmd_vel
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    pub.publish(vel_msg)
    # Faced an error that robot would spiral around after reaching point so loops were divided in this manner to avoid that

# Function for collecting keyboard presses from user and controlling flow of asking for input 
def user_input():

    # Setting global variables to ensure these variables are maintained 
    global goal_point, first_input

    # If loop will keep running while ROS node is running 
    while not rospy.is_shutdown():

        # If first_input is not True so first_input is False 
        # Program will wait for indication that goal is reached and the clear this event to get it ready for next goal values 
        if not first_input:
            goal_reached.wait()  
            goal_reached.clear()
        
        # Then the program will give user an option to enter the values of goal point 
        try:
            goal_point.x = float(input("Input x value for the goal: "))
            goal_point.y = float(input("Input y value for the goal: "))
            
            # Also program will change the first_input to False
            # ensure program waits till robot reaches the point and then will allow user to enter next goal point values 
            if first_input:
                first_input = False

        # If wrong value is entered, like a string, 
        # it will give error and let user to enter both values again         
        except ValueError:
            print("Invalid Value. Input a number.")

# When the first_input logic was missing, program was giving an error 
# as when it runned for the first time, no option was given to enter values 
# as it was waiting for goal_reached event 	


if __name__ == '__main__':
    try:
# Start Node along with having anonymous to True so that it has unique names that don’t have conflict 
        rospy.init_node('auto_nav', anonymous=True)

# Start Publisher that will send message to cmd_vel along with Twist Message type set for it 
# Queue size set to 10 in which it will hold just 10 message at a time 
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

# Start Subscriber which receive info from Pose that hold robot current position 
# callback function is set above and ensure robot location is updated 
        rospy.Subscriber('/turtle1/pose', Pose, callback)

# Sets the to_goal function in move_robot_thread - Allows for movement of robot towards goal point 
# and then starts the execution of thread resulting in function to run         
        move_robot_thread = threading.Thread(target=to_goal)
        move_robot_thread.start()

# Sets the user_input function in user_input - Allows for user to enter goal point values 
# and then starts the execution of thread resulting in function to run 
        user_input_thread = threading.Thread(target=user_input)
        user_input_thread.start()

# .join() ensure that program waits for that thread to complete before moving to next one 
# Allows us to control flow of program 
        move_robot_thread.join()
        user_input_thread.join()

# Program will keep running unless the ROS node is intentionally stopped 
    except rospy.ROSInterruptException:
        pass
