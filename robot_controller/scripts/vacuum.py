#!/usr/bin/env python

#Import rospy for creating nodes in ros
import rospy

# Import the message type which is required for Subscriber to know location of robot
from turtlesim.msg import Pose

# Import the message type which is required for Publisher to give movement commands
from geometry_msgs.msg import Twist

# Setting variable that will hold current point of robot - Initializing Pose in it
current_point = Pose()

# Function used to update robot position 
def callback(pose):

    # Setting current_point as global so it is maintained 
    global current_point

    # Updating current position to current_point variable 
    current_point = pose

    
# Function to ensure vacuum only moves in a linear manner 
# Logic was changed to moving vacuum in only linear due to error of turtle suddenly spiraling continuously all over the window
# Function allows me control movement so it can clean the whole window  
# Parameters - Publishes to cmd_vel to control vacuum along with axis setting direction of movement. 
# target is set for indicating locating vacuum should move to and the speed which is set at 1
def move_line(pub, axis, target, speed=1.0):

    # Setting rate of loop execution - made it higher value to 30 as there is vacuum collision detection requiring faster processing so they don't crash 
    rate = rospy.Rate(30)

    # Initializing Twist in variable to ensure it can communicate with cmd_vel
    vel_msg = Twist()

    # Loop keeps running while ROS node is running 
    while not rospy.is_shutdown():

    # Calculates the absolute value of distance to move based on different between current point and target point 
    # If the axis is x , it will take the differences for values of x. Otherwise, it will take the differences for values of y
        distance = abs(current_point.x - target) if axis == 'x' else abs(current_point.y - target)
        
        # Movement for x axis 
        if axis == 'x':

            # If the point is ahead of vacuum, it will take the positive value of speed. 
            # Otherwise, it will take the negative value of speed to reach the point behind
            # y is kept to 0 as this part of loop only controls x  
            vel_msg.linear.x = speed if current_point.x < target else -speed
            vel_msg.linear.y = 0

        # Movement for y axis 
        elif axis == 'y':

            # speed will be positive for left movement while negative speed will be for right
            # based on whether current point is less than target or not 
            # direction of turning based on positive and negative was found through testing in tele_op script 
            # x is kept to 0 as this part of loop only controls y  
            vel_msg.linear.x = 0
            vel_msg.linear.y = speed if current_point.y < target else -speed

        # Publishes controls to cmd_vel to move vacuum 
        pub.publish(vel_msg)

        # Below loop ensures that if distance is less than 0.1, which mean vacuum is close enough to point 
        # loop will be stopped or will break 
        if distance < 0.1:
            break

        # Will pause for short period to not over pressure the vacuum and allow any pending processes to get cleared 
        rate.sleep()

    # After breaking loop, it will make sure x and y are 0 to stop vacuum - cleanup action 
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0

    # Publish stop controls to vacuum 
    pub.publish(vel_msg)


# Function to make vacuum to move in row or column path 
# Ensure vacuum moves to start point of x and y which is corner of screen 
# Controls are published to cmd_vel 
def pattern(pub, pattern_type, start_x, start_y):

    # if loop for column pattern 
    if pattern_type == 'column':

        # Will keep looping the number of columns in x axis from start point 
        for x in range(start_x, 11):  # Start from the given x position

            # If x divided by 2 is 0 means that x is even number and is at top of screen so target is 0 which is the bottom of screen 
            # if x divided by 2 is not 0 means that x is odd number and is at bottom of screen so target is 11 which is the top of screen
            y_target = 11 if x % 2 != 0 else 0

            # Move vacuum according to y_target set out 
            move_line(pub, 'y', y_target)

            # If x is not equal to 10 (10 being the last column)
            # Vacuum will keep moving to next x axis 
            if x != 10:
                move_line(pub, 'x', x + 1)

            # Prints message once the column is completed 
            rospy.loginfo("Column {} is completed".format(x))

    # if loop for row pattern 
    elif pattern_type == 'row':

        # Will keep looping the number of rows in y axis from start point 
        for y in range(start_y, 11):
            
            # If y divided by 2 is 0 means that y is even number and is at right of screen (end of row) so target is 0 which is the left side of screen (start of row)
            # if y divided by 2 is not 0 means that y is odd number and is at left side of screen (start of row) so target is 11 which is the right side of screen (end of row)
            x_target = 11 if y % 2 != 0 else 0  # Start the first row from x=0

            # Move vacuum according to x_target set out 
            move_line(pub, 'x', x_target)

            # If y is not equal to 10 (10 being the last row)
            # Vacuum will keep moving to next y axis 
            if y != 10:
                move_line(pub, 'y', y + 1)

            # Prints message once the row is completed 
            rospy.loginfo("Row {} is completed".format(y))

    # Prints message once a row or column pattern is completed 
    rospy.loginfo("Pattern {} is completed".format(pattern_type))

if __name__ == '__main__':
    try:

        # Initialize a ROS node and set anonymous for creating unique names 
        rospy.init_node('vacuum', anonymous=True)

        # Start Publisher that will send message to cmd_vel along with Twist Message type set for it 
        # Queue size set to 10 in which it will hold just 10 message at a time 
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Start Subscriber which receive info from Pose that hold robot current position 
        # callback function is set above and ensure robot location is updated 
        sub = rospy.Subscriber('/turtle1/pose', Pose, callback)

        # Wait until first pose message received that so vacuum position is known 
        rospy.wait_for_message('/turtle1/pose', Pose)

        # After completing one set of row and column, the program will loop again but this time, offsetting the position so that it cleans out a different area than the first loop 
        # Setting variables for offsetting row and column along with amount of offset  
        offset_increase = 3
        row_offset = 0
        column_offset = 0

        # Main loop that will be calling all the functions set above while ROS node is running 
        while not rospy.is_shutdown():

            # Starting function for row along with start point for x and y being set to (1,0)
            # Offset will be added accordingly as loop keeps running 
            pattern(pub, 'row', start_x=1 + row_offset, start_y=0 + column_offset)

            # Going to start position for column pattern function and will adjust accordingly to offset and number of loops 
            move_line(pub, 'x', 0 + row_offset)
            move_line(pub, 'y', 1 + column_offset)

            # Starting function for column along with start point for x and y being set to (0,1)
            # Offset will be added accordingly as loop keeps running
            pattern(pub, 'column', start_x=0 + row_offset, start_y=1 + column_offset)

            # Once a loop of pattern function has runned for row and column, the program will increase the offset and assign these values to row_offset and column_offset
            row_offset += offset_increase
            column_offset += offset_increase

            # Whether row OR column offset is greater than 10, both offset values will be resetted to 0 to keep going
            if row_offset >= 10 or column_offset >= 10:
                row_offset = 0
                column_offset = 0

            # Will take a pause for 1 second before starting again 
            rospy.sleep(1)

    # Will keep running until it is stopped or interrupted 
    except rospy.ROSInterruptException:
        pass
