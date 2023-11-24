#!/usr/bin/env python

#Import rospy for creating nodes in ros
import rospy

# Import to aid in flow of code by allowing and blocking certain operations
import threading

# Import the message type which is required for Subscriber to know location of robot
# Allows program to create a flow of operations 
from turtlesim.msg import Pose

# Import service to spawn multiple turtles 
from turtlesim.srv import Spawn

# Import the message type which is required for Publisher
from geometry_msgs.msg import Twist

# NEED COMMENTS 
import time


# Setting global variables for pose (location of robot) and cmd_vel publishes (commands to move robot) 
# Variable is set to be dictionary so it can hold values of multiple vacuums 
poses = {}
vel_pubs = {}

# Function to ensure location of multiple vacuums are updated 
def callback(pose_data, turtle_name):
    global poses

    # Setting location of vacuum to be assigned with the right vacuum 
    poses[turtle_name] = pose_data


# Function to help with avoiding collusion between multiple vacuum 
# Sets the logic of which vacuum should stop and give way to another vacuum by giving priority 
def collusion_priority(turtle_name):


    # Since the vacuums are named turtle1 to turtle4, program will get the 7th value to find the numbering of the turtle and give the output of that integer 
    # The code shows it is taking 6th place as the counting in python starts from 0
    try:
        return int(turtle_name[6:])
    
    # In case there is an value error in which the name is not ending with the number 
    # It will return a 0 value
    except ValueError:
        return 0
    

# Function will help vacuum navigating movement when another vacuum is close and there is chances of collusion 
# Decision will be based on distance between vacuum and number of turtle 
def collusion_movement (turtle_name, min_distance=1.0):

    # Assign pose location based on the vacuum number 
    current_pose = poses[turtle_name]

    # Sets the priority of movement when collision happens based on vacuum number - Initializes the collusion_priority function 
    current_priority = collusion_priority(turtle_name)


    # For loop for checking distance between vacuum and giving priority for movement 
    for other_turtle, other_pose in poses.items():

        # Since we are collecting the pose location and turtle (vacuum) names
        # the for loop is constantly checking whether certain turtles (vacuum) are close to each and whether their names are matching. 
        # If their names are not matching means that two vacuums are close to each other 
        if other_turtle != turtle_name:

            # Calculates the Euclidean between vacuums 
            distance = ((current_pose.x - other_pose.x) ** 2 + (current_pose.y - other_pose.y) ** 2) ** 0.5
            
            # If that distance calculated is less than the minimum distance of 1 
            if distance < min_distance:

                # Check and apply function collusion_priority to decide which vacuum can move 
                other_priority = collusion_priority(other_turtle)

                # 1 has the highest priority and 4 has the lowest 
                # Will check if the current one is smaller then the other vacuum will be stopped 
                if current_priority <= other_priority:
                    return False
                

    # If there is no conflict or chance of collusion - output is True so turtle (vacuum) can move along
    return True

# Same function used in vacuum.py script 
# Function to ensure vacuum only moves in a linear manner 
# Logic was changed to moving vacuum in only linear due to error of turtle suddenly spiraling continuously all over the window
# Function allows me control movement so it can clean the whole window  
# Parameters - Publishes to cmd_vel to control vacuum along with axis setting direction of movement. 
# target is set for indicating locating vacuum should move to and the speed which is set at 1
def move_line(turtle_name, axis, target, speed=1.0):

    # Setting rate of loop execution - made it higher value to 30 as there is vacuum collision detection requiring faster processing so they don't crash
    rate = rospy.Rate(30)

    # Initializing Twist in variable to ensure it can communicate with cmd_vel
    vel_msg = Twist()

	# Loop keeps running while ROS node is running
    while not rospy.is_shutdown():

        # Ensure that the poses location is being assigned with the right vacuum 
        current_pose = poses[turtle_name]

        # Calculates the absolute value of distance to move based on different between current point and target point
	    # If the axis is x , it will take the differences for values of x. Otherwise, it will take the differences for values of y
        distance = abs(current_pose.x - target) if axis == 'x' else abs(current_pose.y - target)

        # First, does the check if vacuum is allowed to move and there is no collusion 
        if collusion_movement(turtle_name):

    	# Movement for x axis
            if axis == 'x':

                # If the point is ahead of vacuum, it will take the positive value of speed.
        	    # Otherwise, it will take the negative value of speed to reach the point behind
        	    # y is kept to 0 as this part of loop only controls x  
                vel_msg.linear.x = speed if current_pose.x < target else -speed
                vel_msg.linear.y = 0

    	    # Movement for y axis
            elif axis == 'y':

                # speed will be positive for left movement while negative speed will be for right
        	    # based on whether current point is less than target or not
        	    # direction of turning based on positive and negative was found through testing in tele_op script
        	    # x is kept to 0 as this part of loop only controls y  
                vel_msg.linear.x = 0
                vel_msg.linear.y = speed if current_pose.y < target else -speed

        # Otherwise, it will just stop the vacuum
        else:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0

	    # Publish stop controls to vacuum - ensures right controls is published to right vacuum 
        vel_pubs[turtle_name].publish(vel_msg)

    	# Below loop ensures that if distance is less than 0.1, which mean vacuum is close enough to point
    	# loop will be stopped or will break
        if distance < 0.1:
            break

        # Will pause for short period to not over pressure the vacuum and allow any pending processes to get cleared
        rate.sleep()

	# After breaking loop, it will make sure x and y are 0 to stop vacuum - cleanup action
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0

	# Publish stop controls to vacuum - ensures right controls is published to right vacuum 
    vel_pubs[turtle_name].publish(vel_msg)


# Function for moving vacuums in the pattern and keep looping 
def pattern_loop(turtle_name, start_x, start_y, x_limit, y_limit, delay):

    # Will delay movement so that they start one after the other 
    time.sleep(delay)

    # Setting the start and limit points of x and y 
    start_x_int = int(start_x)
    start_y_int = int(start_y)
    x_limit_int = int(x_limit)
    y_limit_int = int(y_limit)

    # loop will run while ROS node is running 
    while not rospy.is_shutdown():

        # loop for row pattern as it is running through range of y 
        # Will keep looping the number of rows in y axis from start point untill limit is reached
        for y in range(start_y_int, y_limit_int):

            # If y divided by 2 is not equal to 0, then x is odd and at bottom of screen so the target is set to the limit which is the top of screen 
            # if y divided by 2 is equal to 0, then x is even and at top of screen so the target is set to the start point which is at the bottom of screen 
            x_target = x_limit_int if y % 2 != 0 else start_x_int

            # Move vacuum according to x_target set out 
            move_line(turtle_name, 'x', x_target)

            # If loop will keep the vacuum moving if current y is not less than 1 away from y limit 
            if y != y_limit_int - 1:
                move_line(turtle_name, 'y', y + 1)
        
        # loop for column pattern as it is running through range of s
        for x in range(start_x_int, x_limit_int):

            # if x divided by 2 is not 0 means that y is odd number and is at left side of screen (start of row) so target is set to limit which is the right side of screen (end of row)
            # If x divided by 2 is 0 means that y is even number and is at right of screen (end of row) so target is set to start point point which is the left side of screen (start of row)
            y_target = y_limit_int if x % 2 != 0 else start_y_int

            # Move vacuum according to y_target set out 
            move_line(turtle_name, 'y', y_target)

            # If loop will keep the vacuum moving if current x is not less than 1 away from x limit 
            if x != x_limit_int - 1:
                move_line(turtle_name, 'x', x + 1)


# Function for spawning vacuum ( turtles) using the ROS service 
def spawn_turtle(turtle_name, x, y, theta):

    # Will wait for the required service to be ready 
    rospy.wait_for_service('spawn')

    try:

        # Sets up Service Proxy to be able to call spawn service 
        spawner = rospy.ServiceProxy('spawn', Spawn)

        # Will spawn turtles - name and location variables are defined 
        spawner(x, y, theta, turtle_name)

        # Print successful message is spawn is done and give location of that vacuum 
        rospy.loginfo(f"Spawned turtle {turtle_name} at x={x}, y={y}, theta={theta}")

    # loop will be stopped if there is an error in calling service 
    except rospy.ServiceException as error:

        # Error message will be printed 
        rospy.logerr("Service call failed: %s" % error)


# Function starts the loop for vacuum in thread to have smooth flow of operation 
# Parameters with turtle name with start and limits and the delay set 
def start_pattern_loop(turtle_name, start_x, start_y, x_limit, y_limit, delay):
    
    # threading the event for turtle movement with using pattern_loop function 
    thread = threading.Thread(target=pattern_loop, args=(turtle_name, start_x, start_y, x_limit, y_limit, delay))
    
    # Setting it as daemon which runs in the background and closes when program has ended 
    thread.daemon = True

    # Start thread which also activates pattern_loop function 
    thread.start()

if __name__ == '__main__':
    try:
        # Initialize a ROS node and set anonymous for creating unique names 
        rospy.init_node('vacuum_multi', anonymous=True)


        # Setting mid points - will be used to divide quadrants 
        x_mid = 5.5
        y_mid = 5.5

        # Spawning 3 turtles only as when you start turtlesim node, turtle1 is already spawned
        # If you spawn turtle1, program will face an error in which service call will be killed as that turtle already exist 
        # Parameters is naming turtle and setting x and y position along with orientation 

        # Placed in Quadrant 2 - Top Left from MidPoint 
        spawn_turtle("turtle2", x_mid + 1, 1, 0)

        # Placed in Quadrant 3 - Bottom Right from MidPoint 
        spawn_turtle("turtle3", 1, y_mid + 1, 0)

        # Placed in Quadrant 4 - Top Right from MidPoint 
        spawn_turtle("turtle4", x_mid + 1, y_mid + 1, 0)

        # For loop to hold Pose ( Location of turtle) and Publisher to cmd_vel ( To move Turtle ) 
        for i in range(1, 5):

            # Initializing dictionary with turtle_name
            turtle_name = f"turtle{i}"

            # Initializing pose according to turtle name 
            poses[turtle_name] = Pose()

            # Start Publisher that will send message to cmd_vel along with Twist Message type set for it
            # Queue size set to 10 in which it will hold just 10 message at a time
            # Loop to ensure that publisher is started for all 4 turtles by looping turtle name 
            vel_pubs[turtle_name] = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
            
            # Start Subscriber which receive info from Pose that hold robot current position
    	    # callback function is set above and ensure robot location is updated according to turtle name ( callback_args )
            # Loop to ensure that subscriber is started for all 4 turtles by looping turtle name
            rospy.Subscriber(f'/{turtle_name}/pose', Pose, callback, callback_args=turtle_name)
            
            # Wait for pose message of turtles spawn 
            # WIll not wait for Turtle1 as it is known that it already exist
            if i != 1:
                rospy.wait_for_message(f'/{turtle_name}/pose', Pose)



        # Starts the pattern loop for cleaning - Calling threads for each turtle using start_pattern_loop which call pattern_loop function to move vacuum in loop 
        # Parameters are those defined in start_pattern_loop function which is turtle name with start and limit of x and y along with delay 

        # Placed in Quadrant 1 - Bottom Left from MidPoint   
        start_pattern_loop('turtle1', 0, 0, x_mid, y_mid, 0)        
        
        # Placed in Quadrant 2 - Top Left from MidPoint   
        start_pattern_loop('turtle2', x_mid, 0, 11, y_mid, 0)       
        
        # Placed in Quadrant 3 - Bottom Right from MidPoint   
        start_pattern_loop('turtle3', 0, y_mid, x_mid, 11, 0)      
        
        # Placed in Quadrant 4 - Top Right from MidPoint 
        start_pattern_loop('turtle4', x_mid, y_mid, 11, 11, 0)      

        # Ensures that node is running until it is stopped or shutdown 
        rospy.spin()

    # Will stop if user has interrupted or stopped program 
    except rospy.ROSInterruptException:
        pass
