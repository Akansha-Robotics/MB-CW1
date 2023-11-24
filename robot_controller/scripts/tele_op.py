#!/usr/bin/env python

#Import rospy for creating nodes in ros
import rospy

# Import the message type which is required for Publisher
from geometry_msgs.msg import Twist

# Import various libraries to use keyboard press for controlling robot 

import sys, select, termios, tty

# sys = read keyboard presses which is collected through sys.stdin 
# select = waits to receive incoming data from sys.stdin and returns indication when it is pressed 
# termios and tty = used to control terminal setting and behavior so that when user presses a key, 
# that information can be collected immediately rather than waiting for user to press enter to send command 
# allows keyboard press to be passed to programing without any preprocessing enabling quick and immediate control 

# Print message for user on key options 
keys_msg = """
Control Options for Robot 

Forward - W 
Backward - S
Left - A
Right - D
Quit - Q

For speed change, press + to get option to input updated speed 

"""

# Binding the movements according keys pressed 
# Forward and backward will change X = Linear X 
# Left and right will change Y - Angular Z 
moveBindings = {
    'w': (1, 0),
    'a': (0, 1),
    's': (-1, 0),
    'd': (0, -1),
}
# Decided to go with binding as entering key press for each movement will be frustrating and slow process for user 

# Function below reads every individual keyboard press and does not wait for a new line of command to process control.
def key_input (settings):

    # Ensure terminal behavior is changed to raw mode - tty.setraw
    # Give the keyboard press as a file descriptor which is then used to interact with python to control robot - sys.stdin.fileno()
    tty.setraw(sys.stdin.fileno())

    # Takes 3 file descriptors from sys.stdin and waits for first 1 to be ready to read it 
    # Waiting for input to ready is done by select module 
    # 0 means that there is no wait time and nothing blocks it 
    select.select([sys.stdin], [], [], 0)

    # Will read just 1 keyboard press from sys.stdin and returns command immediately 
    key = sys.stdin.read(1)

    # Function changes to raw mode terminal setting for accepting keyboard presses and then returns it back to normal; 
    # termios.tcsetattr - setting terminal behavior or parameters 
    # First, calling terminal setting for accommodating sys.stdin
    # Second, termios.TCSADRAIN temporarily changing terminal setting to ensure all output is being sent(ensures there are no issues caused by input or output as we are changing settings)
    # Third, will apply the original setting for terminal 
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    # gives output of key that was read
    return key

if __name__=="__main__":

    # Saving the original setting of terminal before doing any change - will be used to restore the setting back to normal 
    settings = termios.tcgetattr(sys.stdin)

    # Initialize node and set anonymous to True so it creates unique names  
    rospy.init_node('tele_op', anonymous=True)

    # Start Publisher that will send message to cmd_vel along with Twist Message type set for it 
    # Queue size set to 10 in which it will hold just 10 message at a time 
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

    # Setting default speed for moving robot in a linear and angular manner
    linear_speed = 0.5
    angular_speed = 1.0


    try:

        # Prints the message for user on the keys they can use to control robot 
        print(keys_msg)

        # While loop indicates that program should keep running until a break is indicated 
        while True:

            # Calling the function (that would detect keyboards presses and return the key that was pressed) and saves the commands into the variable called key
            key = key_input(settings)

            # if the keyboard press matches the keys listed on moveBindings, it will take action accordingly 
            if key in moveBindings.keys():

                # For linear, it will access the first element of the tuple of moveBindings for that key 
                linear_cmd = moveBindings[key][0]

                # For angular, it will access the second element of the tuple of moveBindings for that key 
                angular_cmd = moveBindings[key][1]

            # if the + button has been press, it will give users an option to edit the linear speed to the new value 
            elif key == '+':
                try:
                    # Prompts user to enter new speed value
                    new_speed = float(input("Enter speed value: "))
                    
                    # linear_speed variable gets updated with new value entered by user 
                    linear_speed = new_speed

                # if the values is not a float value, then it will print an error message 
                except ValueError:
                    print("Invalid values. Speed has not been adjusted")
            
            # if the q key is press, then it will break or stop program 
            elif key == 'q':
                break
            
            # Any other key that is not in moveBindings and is not q nor +, no action will be taken and robot will stay in same place 
            else:
                linear_cmd = 0
                angular_cmd = 0

            # Initialize Twist in variable to be able to communicate with cmd_vel
            twist = Twist()

            # Calculates the final values for linear and angular movement by taking the direction movement and multiplying by speed 
            
            # If linear speed has been changed, it will adjust accordingly 
            twist.linear.x = linear_cmd * linear_speed

            # For angular, the program is only allowing the speed set by program which is 0.5 as changing these values to be too extreme can make the robot difficult to control 
            twist.angular.z = angular_cmd * angular_speed 

            # the values of twist will then be published to move robot 
            pub.publish(twist)

    # If there is any error, it will block the try program loop and print the error 
    except Exception as error:
        print(error)

    # finally will be executed no matter how the try program ended - clean up action 
    finally:

        # Initialize Twist in variable to be able to communicate with cmd_vel
        twist = Twist()

        # Movement controls set to 0 to stop robot 
        twist.linear.x = 0
        twist.angular.z = 0

        # Publishes movement controls to cmd_vel
        pub.publish(twist)

        # Change terminal setting back to normal and original one 
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
