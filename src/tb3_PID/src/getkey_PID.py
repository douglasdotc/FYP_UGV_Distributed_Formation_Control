#!/usr/bin/env python

# Copy from: https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop/nodes/turtlebot3_teleop_key
# http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal

import sys, select, os
import rospy, tf
import csv
import time
import matplotlib.pyplot            as plt
import numpy                        as np
import networkx                     as nx

from geometry_msgs.msg              import Twist, Pose2D
from nav_msgs.msg                   import Odometry
from math                           import radians, pi, sqrt, asin, acos, atan2, sin, cos, tan
from datetime                       import datetime
from matplotlib.animation           import FuncAnimation
from AStar                          import astar
from quintic_polynomials_planner    import quintic_polynomials_planner
from potential_field_planning       import potential_field_planning


# Global Constant
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE  = 0.01
ANG_VEL_STEP_SIZE  = 0.1

# Global Variable
g_start_time        = time.time()   # float with unit [second]
g_num_of_robot      = 0             # Total number of robots
g_multi_robot       = []            # Robot names
g_adj_mat           = []            # Adjacency Matrix
g_zeta              = []            # Agent understanding of of the center position of the formation, in vector form [x, y, theta].T
g_zeta_dot          = []            # Derivative of g_zeta
g_r_rel             = []            # Relative distance to virtual leader / center of formation
g_ref_state_contr   = []            # Reference state / virtual leader position / center of formation
g_additional_radius = []            # Additional radius (Time varying formation)
g_feedback_pose     = {}            # Feedback pose


class Pose():
    """
        Description:
            Robot pose, describe by x, y position and angle theta
    """

    def __init__(self):
        self.x     = 0.0 # x
        self.y     = 0.0 # y
        self.theta = 0.0 # theta

class PID():
    """
    Description:
            PID controller        
        Functions:
            def __init__(self, dt, Kp, Kd ,Ki, max, min)
            def cal(self, setpt, fb)
            def clear_param(self)

    """
    def __init__(self, dt, Kp, Kd ,Ki, max, min):
        """
            Description:
                Initialize PID controller parameters

        """
        self.Kp        = Kp    # Proportional Gain
        self.Ki        = Ki    # Integral Gain
        self.Kd        = Kd    # Derivative Gain
        self.dt        = dt    # Sample Time
        self.max       = max   # Maximum output
        self.min       = min   # Minimum output

        self.Iout      = 0     # Integral output
        self.pre_error = 0     # Previous error
    
    def cal(self, setpt, fb):
        """
            Description: 
                Calculate the output of PID controller
                **Output is bounded by the maximum and minimum value**
        """
        # Error: Set position - feedback position
        error = setpt - fb

        # Proportional output
        Pout = self.Kp * error

        # Derivative output
        Dout = self.Kd * (error - self.pre_error) / self.dt

        # Integral output
        self.Iout += self.Ki * error * self.dt

        # Total output
        output = Pout + Dout + self.Iout

        # Limit total output to max / min
        if( output > self.max ):
            output = self.max

        elif( output < self.min ):
            output = self.min

        # Save error to previous error
        self.pre_error = error

        return output

    def clear_param(self):
        """
            Description:
                Reset Integral output and error to reinitiate Burger
        """
        self.Iout      = 0
        self.pre_error = 0

class TurtleBot():
    """
        Description:
            Single Robot to be control

    """

    def __init__(self, ctrlfreq, multi_robot_name = '', datalog = False):
        """
            Description:
                Initialize:
                    Robot name prefix: 
                        Example: 
                            TurtleBot3 number 1 --> 'tb3_1/'
                            TurtleBot3 number 3 --> 'tb3_3/'

                    Sampling time parameters
                    Status of Motion, Feedback and Datalog
                    Pose and velocity message
                    Positional tolerance
                    Respective neighbour robot distance and self robot position
                    PID Controller

        """
        if not multi_robot_name == '':
            multi_robot_name += '/'

        self.prefix_robot = multi_robot_name

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(self.prefix_robot + 'cmd_vel', Twist, queue_size = 10)
        print(self.prefix_robot, 'subscriber')

        self.goalpose_subcriber = rospy.Subscriber(self.prefix_robot + 'goal_pose', Pose2D, self.callback_goalpose)
        self.mocap_subscriber   = rospy.Subscriber(self.prefix_robot + 'ground_pose', Pose2D, self.callback_mocap, queue_size=1)

        # Sampling Time
        self.ctrlfreq = ctrlfreq                    # Control frequency
        self.dt       = 1.0/self.ctrlfreq           # Control period
        self.rate     = rospy.Rate(self.ctrlfreq)   # Control sleeping rate

        turtlebot3_model = rospy.get_param("model", "burger")

        # Status of Motion, Feedback and Datalog
        self.flag_mot       = 0         # Boolean motion flag:      0 --> ready,    1 --> moving
        self.flag_fb        = 0         # Boolean feedback flag:    0 --> offline,  1 --> online
        self.flag_datalog   = datalog   # Boolean datalog flag:     0 --> disable,  1 --> enable
        self.flag_quintic_planner = False

        '''
        if self.flag_datalog == True:
            self.prepare_df()
        '''
        # Pose and velocity message (?)
        self.pose        = Pose()       # Pose
        self.lastpose    = Pose()       # Last pose
        self.velmsg      = Twist()      # velocity in terms of (v, w)

        # Positional tolerance
        self.linear_tol  = 0.1          # Linear position tolerance
        self.angular_tol = radians(5)   # angular position tolerance

        # Respective neighbour robot distance and self robot position
        self.neighbor_distance = []     # Distances between self and neighbour robots from 1 to n
        self.turtle_position   = []     # Self robot position
        
        # Local Consensus
        global g_num_of_robot
        self.pre_ref_state_contr = np.zeros(3)   # Stored control reference state [x, y, theta]
        self.pre_ref_state       = []            # Stored agents understandings of reference states (3 x N matrix)
        self.epsilon             = []            # All agents' understanding of the maximum and minimum distances in the formation (4 x N)

        """
        PID schemes for [p,a,b]
            Input: (dt, Kp, Kd ,Ki, max, min)
        """
        self.pid_p = PID(self.dt, 1, 0, 0.001, 0.1, -0.1)
        self.pid_a = PID(self.dt, 10, 0, 0.05, pi, -pi)
        self.pid_b = PID(self.dt, -5, 0, -0.005, pi, -pi)
        self.Ki_output = 0      # Ignore integral feedback for now

        self.path            = []
        self.maze            = []
        self.numofstep       = 0
        self.currentstep     = 0
        self.path_calculated = 0
        self.x_path          = []
        self.y_path          = []

        self.flag_consensus = False

    #####################################################################
    # Helper Functions
    def callback(self, data):
        """
            Description:
                **
                Used in Subscriber 
                    (Class for registering as a subscriber to a specified topic, 
                     where the messages are of a given type.)
                **
                Subscriber will call this function when data is received

                This function update positional parameters and twist (velocity ?) at time t - 1 (last pose) and t (goal pose):
                    Parameters at   t - 1 = t
                    Parameters at   t     = t + 1 (next position)

        """
        #print(data)
        # Assign positional parameters at   t - 1 = t
        self.lastpose.x     = self.pose.x
        self.lastpose.y     = self.pose.y
        self.lastpose.theta = self.pose.theta

        # Assign positional parameters at   t     = t + 1 (next position)
        self.pose.x     = data.pose.pose.position.x
        self.pose.y     = data.pose.pose.position.y
        self.pose.theta = data.pose.pose.orientation.z * pi / 1.55 # Map (-1, +1) to (-pi, pi)

        # Update velocity
        self.twist = data.twist.twist

    def callback_goalpose(self, data):
        """
            Description:

        """
        self.goal_pose = data
        self.flag_mot  = 1
        print(self.prefix_robot,'received new pose')

        if self.flag_consensus:
            global g_ref_state_contr, g_additional_radius
            if g_ref_state_contr.ndim > 1:
                self.Consensus(g_ref_state_contr[0], g_additional_radius[0])
            elif g_ref_state_contr.ndim == 1:
                self.Consensus(g_ref_state_contr, [])
        else:
            if self.flag_quintic_planner:
                self.time = time.time()
                self.mot_time, self.mot_x, self.mot_y, self.mot_theta, v, a, j = quintic_polynomials_planner(self.pose, self.goal_pose)
                self.P2P_quintic()
            else:
                self.P2P_mocap_noPlanning()

    def callback_mocap(self, data):
        """
            Description:
                Subscriber will call this function when mocap message is received
        """
        global g_start_time, g_ref_state_contr, g_additional_radius
        mocap_pose = data
        self.update_pose(mocap_pose)

        if self.flag_quintic_planner and hasattr(self, 'mot_time'):
            self.P2P_quintic()

        # Time varying formation
        if self.flag_consensus and g_ref_state_contr.ndim > 1:
            time_               = time.time()
            time_started        = time_ - g_start_time
            time_for_each_state = 2     # Time for each start in second
            num_form_pt         = int(time_started/time_for_each_state) 

            if (num_form_pt < (len(g_ref_state_contr) - 1)):
                self.flag_mot = 1
            else:
                num_form_pt = len(g_ref_state_contr) - 1

        if self.flag_mot == 1:
            if self.flag_consensus:
                if g_ref_state_contr.ndim > 1:
                    self.Consensus(g_ref_state_contr[num_form_pt], g_additional_radius[num_form_pt])
                elif g_ref_state_contr.ndim == 1:
                    self.Consensus(g_ref_state_contr, [])
            else:
                self.P2P_mocap_noPlanning()

    def update_pose(self, pose_fb):
        """
            Description:
            **
            Used in Subscriber 
                (Class for registering as a subscriber to a specified topic, 
                 where the messages are of a given type.)
            **
            Subscriber will call this function when a new Pose is received
            And if data log flag is on, log the current information
        """

        # Store pose of last sample
        self.lastpose.x     = self.pose.x
        self.lastpose.y     = self.pose.y
        self.lastpose.theta = self.pose.theta

        self.pose.x     = round(pose_fb.x, 4)
        self.pose.y     = round(pose_fb.y, 4)

        pose_fb.theta   = self.angle_bound(pose_fb.theta - pi/2) # Angle alignment
        self.pose.theta = round(pose_fb.theta, 4)

        #print(self.prefix_robot)
        global g_multi_robot, g_num_of_robot, g_feedback_pose
        
        TurtleIndex = self.prefix_robot[0:5]
        g_feedback_pose[TurtleIndex].x = self.pose.x
        g_feedback_pose[TurtleIndex].y = self.pose.y
        g_feedback_pose[TurtleIndex].theta = self.pose.theta


        # If data log flag is on, log the current information
        if (self.flag_datalog == True) and (hasattr(self, 'goal_pose')):
            self.update_df()

    def set_zerovel(self):
        """
            Description:
                Set zero for linear and angular velocity
        """
        self.velmsg.linear.x  = 0   # v
        self.velmsg.linear.y  = 0   # --
        self.velmsg.linear.z  = 0   # --
        self.velmsg.angular.x = 0   # --
        self.velmsg.angular.y = 0   # --
        self.velmsg.angular.z = 0   # w

    def set_zeropose(self):
        """
            Description:
                Set zero for positional parameters

        """
        self.pose.x         = 0     # x     @ t
        self.pose.y         = 0     # y     @ t
        self.pose.theta     = 0     # theta @ t
        self.lastpose.x     = 0     # x     @ t - 1
        self.lastpose.y     = 0     # y     @ t - 1
        self.lastpose.theta = 0     # theta @ t - 1

    def clear_PID_accu(self):
        """
            Description:
                Reset PID controller (?)

        """
        self.pid_p.clear_param()
        self.pid_a.clear_param()
        self.pid_b.clear_param()
        self.Ki_output = 0

    def update_df(self):
        """
            Description:
                Log current information to the file: self.dir_datalog: dir_parent + '/motion_' + self.prefix_robot[:-1] + '.txt', 
                including following:
                    Time
                    Goal positional parameters
                    Current positional parameters
                    Motion Status

        """
        time         = datetime.now().strftime('%Y%m%d_%H%M%S.%f')
        datalog_list = [time, 
                        self.goal_pose.x,     self.pose.x, 
                        self.goal_pose.y,     self.pose.y,
                        self.goal_pose.theta, self.pose.theta, 
                        self.flag_mot]

        with open(self.dir_datalog, 'a') as file_:
            for word in datalog_list:
                file_.write(str(word) + '\t')

            file_.write('\n')

    def update_dirdatalog(self, dir_parent):
        """
            Description:
                Update Log file with headings in datalog_list in this function

        """
        self.dir_datalog = dir_parent + '/motion_' + self.prefix_robot[:-1] + '.txt'
        datalog_list     = ["Time", "X_Setpt", "X_FB", "Y_Setpt", "Y_FB", "Theta_Setpt", "Theta_FB", "Motion Status"]

        with open(self.dir_datalog, 'w+') as file_:
            for word in datalog_list:
                file_.write(str(word) + '\t')
            file_.write('\n')

    def pub_vel(self):
        """
            Description:
                Publish linear and angular velocity

        """
        self.velocity_publisher.publish(self.velmsg)

    def cal_linear_vel(self, currvel, velsetpt):
        """
            Description:
                Calculate PID controlled linear velocity

                **CURRENTLY NOT IN USE**
        """
        self.velmsg.linear.x = self.pidVelocity.cal(velsetpt, currvel)

    def euclidean_distance(self, goal_pose):
        """
            Description:
                Calculate and return euclidean distance between current pose and the goal

        """
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def angle_bound(self, ang):
        """
            Description:
                Return an angle between [-pi, pi] for control and coordinate transformation purpose (cartesian to polar)

        """
        while ang > pi:
            ang -= 2*pi

        while ang < -pi:
            ang += 2*pi

        return ang 
  
    def steering_angle(self, goal_pose):
        """
            Description:
                This function calculate and return steering angle

                Reference to "Robotics Vision and Control (2017)" Page 107:
                    4.1.1.4 Moving to a Pose

                Steering angle is defined as the angle between the orientation of 
                the robot and the orientation of the rotatable front wheels, 
                which is parallel to the direction pointing to the goal pose.

                Unlike the formula atan(w*L/v) in the book, our robots have 
                two wheels only and can just move forward / backward. 

                Therefore, we self-defined the steering angle as 
                the angle between the orientation of the robot and x-axis:

                    steering angle = atan(dy/dx)

        """
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def pathtracking_simple(self, goal_pose):
        """
            Description:
                Simple tracking as mentioned in Interim Report
                Has better performance comparing to move_to_pose

                Since TurtleBot3 Burger takes input of linear and angular velocities,
                we could add PI control in position loop to mimic PIV control.
        """
        dist         = self.euclidean_distance(goal_pose)      # p                 (Direct distance between the start pose and goal pose)
        ang          = self.steering_angle(goal_pose)
        ang_deviated = self.angle_bound(ang - self.pose.theta)
        linear_gain  = 0.1
        angular_gain = 0.5
        # Linear Velocity, only move forward
        #linear_vel_setpt = min(dist/self.dt, 0.10) # unit: m/s
        linear_vel_setpt = linear_gain * dist/self.dt # unit: m/s
        # if vel is deviated, punish vel with cos
        #linear_vel_setpt *= cos(ang_deviated / 2)

        # This part tries to mimic PI control but needed to adjust
        # currvel = self.euclidean_distance(self.lastpose) / self.dt
        # self.cal_linear_vel(currvel, linear_vel_setpt)

        if abs(ang_deviated) > self.angular_tol:
            #angular_vel_setpt = min(abs(ang_deviated/self.dt), radians(45)) * ang_deviated / abs(ang_deviated) # unit: rad/s
            angular_vel_setpt = angular_gain * ang_deviated/self.dt # unit: rad/s
        else:
            angular_vel_setpt = 0

        return linear_vel_setpt, angular_vel_setpt
       
    def move_to_pose(self, goal_pose):
        """
            Description:
                Calculate and return linear and angular velocity (PI control).

                Reference to "Robotics Vision and Control (2017)" Page 107:
                    4.1.1.4 Moving to a Pose
                
                Linear Velocity  = (k_p)*p              (linear_vel_setpt)
                Angular velocity = (k_a)*a + (k_b)*b    (angular_vel_setpt)

        """
        kp = 0.8   # k_p
        ka = 5     # k_a
        kb = -2.5  # k_b

        #  Parameters for quintic polynomials planner
        if self.flag_quintic_planner:
            kp = 0.3
            ka = 6
            kb = -3
            # kp = 0.15
            # ka = 1
            # kb = -1.25

        dist         = self.euclidean_distance(goal_pose)                                   # p                 (Direct distance between the start pose and goal pose)
        ang          = self.steering_angle(goal_pose)                                       # a + theta         (Angle between p and x-axis, anti-clockwise)
        ang_deviated = self.angle_bound(ang - self.pose.theta)                              # a                 (Angle between p and the current pose orientation, anti-clockwise)
        ang_beta     = self.angle_bound(- self.pose.theta - ang_deviated + goal_pose.theta) # b = d theta - a   (angle between the direction of p and the goal orientation, clockwise.
                                                                                            #                    dtheta for accomodating the case where goal orientation != 0)

        # When robot far from goal pose
        if dist > self.linear_tol:
            """
            If p > Linear positional tolerance:

                Linear Velocity = (k_p)*p
            
            Leading to: 
                As p --> 0; Linear Velocity --> 0.
            """

            linear_vel_setpt = kp * 0.25#* dist
            if self.flag_quintic_planner:
                linear_vel_setpt = kp * dist
            angular_vel_setpt = ka * ang_deviated + kb * ang_beta

            return linear_vel_setpt, angular_vel_setpt
        
        else:
            linear_vel_setpt = 0

        print('goal_pose.theta: ',goal_pose.theta, ',self.pose.theta: ',self.pose.theta)
        print('mot_theta:',self.mot_theta[-1])

        # When robot near goal pose
        if abs(goal_pose.theta-self.pose.theta) > self.angular_tol:
            angular_vel_setpt = ka * ang_deviated + kb * ang_beta
        else:
            angular_vel_setpt = 0

        return linear_vel_setpt, angular_vel_setpt

    ##########################################################
    # Motion Functions
    def stop(self):
        """
            Description:
                Stop robot motion
        """
        print('stop one')
        self.set_zerovel()
        self.pub_vel()
        self.flag_mot = 0

    def P2P_mocap_noPlanning(self):
        """
            Description:
                Point to point path motion
                self.pose and self.goal_pose needed to be ready before calling this function
                Feedback of pose will not be updated in this function
        """

        # Get updated situation
        self.goal_pose.theta = self.angle_bound(self.goal_pose.theta)
        dist                 = self.euclidean_distance(self.goal_pose)   # p             (Direct distance between the start pose and goal pose)
        ang                  = self.steering_angle(self.goal_pose)       # a + theta     (Angle between p and x-axis, anti-clockwise)
        ang_deviated         = self.angle_bound(ang - self.pose.theta)
        theta_diff           = self.goal_pose.theta - self.pose.theta

        global g_multi_robot, g_num_of_robot, g_feedback_pose

        if (not rospy.is_shutdown()) and ((dist > self.linear_tol)):
            # Assign Linear and angular velocity of self robot required for publishment
            linvelsetpt, angvelsetpt = self.pathtracking_simple(self.goal_pose)
            self.velmsg.linear.x     = linvelsetpt
            self.velmsg.angular.z    = angvelsetpt

            # Publish required velocity profile (v, w)
            self.pub_vel()


        elif (not rospy.is_shutdown()) and (abs(theta_diff) > self.angular_tol):
            linvelsetpt  = 0
            theta_diff   = self.angle_bound(self.goal_pose.theta - self.pose.theta)
            angvelsetpt  = 0.5 * theta_diff / abs(theta_diff) + pi/10 * theta_diff

            # Assign Linear and angular velocity of self robot required for publishment
            self.velmsg.linear.x  = linvelsetpt
            self.velmsg.angular.z = angvelsetpt

            # Publish required velocity profile (v, w)
            self.pub_vel()

        else:
            # Robot stop if linear and angular tolerances are met
            self.stop()
            print(self.prefix_robot + ' reach target pose')

        self.rate.sleep()

    def P2P_mocap_CollisionAvoid(self):
        """
            Description:
                Test for collision avoidance

                **NOT IN USE**
        """
        current_pose = Pose2D()
        current_pose.x = self.pose.x
        current_pose.y = self.pose.y
        current_pose.theta = self.pose.theta
        if (self.prefix_robot[:-1] == 'tb3_1'): 
            self.PotentialField_PathPlanning(current_pose, self.goal_pose)

    def PotentialField_PathPlanning(self, start_pose, goal_pose):
        """
            Description:
                Path Planning using Potential Field
                Used by P2P_mocap_CollisionAvoid()
        """
        sx           = round(start_pose.x,1)  # start x position [m]
        sy           = round(start_pose.y,1)  # start y positon [m]
        gx           = round(goal_pose.x, 1)  # goal x position [m]
        gy           = round(goal_pose.y, 1)  # goal y position [m]
        grid_size    = 0.1                    # potential grid size [m]
        robot_radius = 0.3                    # robot radius [m]

        ox = [round(g_feedback_pose['tb3_2'].x,1),round(g_feedback_pose['tb3_4'].x,1), round(g_feedback_pose['tb3_6'].x,1)]  # obstacle x position list [m]
        oy = [round(g_feedback_pose['tb3_2'].y,1), round(g_feedback_pose['tb3_4'].y,1), round(g_feedback_pose['tb3_6'].y,1)]  # obstacle y position list [m]

        print(self.path_calculated)
        if self.path_calculated == 0:
           print('calculating path...')
           x, y = potential_field_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
           print(x, y)
           self.path_calculated = 1
           self.x_path = x
           self.y_path = y
           self.numofstep = len(x)
           self.currentstep = 0        


        next_goal        = Pose2D()
        next_goal.x      = self.x_path[self.currentstep]
        next_goal.y      = self.y_path[self.currentstep]
        ultimate_goal    = Pose2D()
        ultimate_goal.x  = self.x_path[self.numofstep-1]
        ultimate_goal.y  = self.y_path[self.numofstep-1]
        self.goal_pose.x = next_goal.x
        self.goal_pose.y = next_goal.y

        dist_goal          = self.euclidean_distance(next_goal)
        dist_ultimate_goal = self.euclidean_distance(ultimate_goal)

        print(ultimate_goal, self.currentstep)
        if (dist_goal < self.linear_tol) and (dist_ultimate_goal < self.numofstep - 1):
            self.currentstep += 1
       
        if (not rospy.is_shutdown()) and ((dist_ultimate_goal > self.linear_tol)):
            # Assign Linear and angular velocity of self robot required for publishment
            linvelsetpt, angvelsetpt = self.pathtracking_simple(next_goal)
            self.velmsg.linear.x     = linvelsetpt
            self.velmsg.angular.z    = angvelsetpt
            print('P2P mocap pub vel', self.velmsg.linear.x, self.velmsg.angular.z)

            # Publish required velocity profile (v, w)
            self.pub_vel()
        else:
            # Robot stop if linear and angular tolerances are met
            self.stop()
            print(self.prefix_robot + ' reach target pose')
            
    def AStar_PathPlanning(self, start_pose, goal_pose):
        """
            Description:
                A* Search Algorithm Path Planning
                
        """
        self.maze = []                      

        # Update collision map
        sizeofmap = 250

        # Map init
        for i in range(0,sizeofmap):
            map_elements = []
            for j in range(0,sizeofmap):
                map_elements.append(0)
                self.maze.append(map_elements)        

        coordinates_shift = 150
        multiplier        = 50.0
        turtle_radius     = 10

        start   = (int(multiplier*start_pose.x) + coordinates_shift, int(multiplier*start_pose.y) + coordinates_shift)
        end     = (int(multiplier*goal_pose.x) + coordinates_shift, int(multiplier*goal_pose.y) + coordinates_shift)
        path    = astar(self.maze, start, end)

        next_goal = Pose2D()
        if len(path) > 1:
            next_goal.x     = (path[1][0] - coordinates_shift)/multiplier
            next_goal.y     = (path[1][1] - coordinates_shift)/multiplier
            next_goal.theta = goal_pose.theta
            dist            = self.euclidean_distance(next_goal)   # p             (Direct distance between the start pose and goal pose)
            ang             = self.steering_angle(next_goal)       # a + theta     (Angle between p and x-axis, anti-clockwise)
            ang_deviated    = self.angle_bound(ang - next_goal.theta)         

        self.goal_pose.theta = self.angle_bound(self.goal_pose.theta)
        dist_goal            = self.euclidean_distance(self.goal_pose)
        
        if (not rospy.is_shutdown()) and ((dist_goal > self.linear_tol)):
            # Assign Linear and angular velocity of self robot required for publishment
            linvelsetpt, angvelsetpt = self.pathtracking_simple(next_goal)
            self.velmsg.linear.x     = linvelsetpt
            self.velmsg.angular.z    = angvelsetpt/2
            # Publish required velocity profile (v, w)
            self.pub_vel()
        else:
            self.stop()
            print(self.prefix_robot + ' reach target pose')

    def P2P_quintic(self):

        """
            Description:
                Quintic Polynomial Path planning
        """
        self.linear_tol  = 0.05
        self.angular_tol = radians(5)

        if not rospy.is_shutdown():
            # Find the corresponding goal pose
            time_now = time.time() - self.time
            if time_now > self.mot_time[-2]:
                num = len(self.mot_time) - 1
            else:
                value = min(self.mot_time, key = lambda x:abs(x - time_now))
                num   = self.mot_time.index(value)
                if value < time_now:
                    num += 1

            goal_pose       = Pose2D()
            goal_pose.x     = self.mot_x[num]
            goal_pose.y     = self.mot_y[num]
            goal_pose.theta = self.mot_theta[num]
            self.goal_pose  = goal_pose

            print('P2P_quintic num:',num)

            linvelsetpt, angvelsetpt = self.move_to_pose(goal_pose)
            self.velmsg.linear.x     = linvelsetpt
            self.velmsg.angular.z    = angvelsetpt

            # Publish required velocity profile (v, w)
            self.pub_vel()

        self.rate.sleep()
    
    def P2P_mocap(self):
        """
            Description:
                Point to point path motion (20200323 testing)
                self.pose and self.goal_pose needed to be ready before calling this function
                Feedback of pose will not be updated in this function

                **NOT IN USE**
        """

        # Get updated situation
        dist         = self.euclidean_distance(self.goal_pose)   # p             (Direct distance between the start pose and goal pose)
        ang          = self.steering_angle(self.goal_pose)       # a + theta         (Angle between p and x-axis, anti-clockwise)
        ang_deviated = self.angle_bound(ang - self.pose.theta)

        global g_multi_robot, g_num_of_robot, g_feedback_pose   

        if (not rospy.is_shutdown()) and ((dist > self.linear_tol)):
            # Assign Linear and angular velocity of self robot required for publishment
            linvelsetpt, angvelsetpt = self.pathtracking_simple(self.goal_pose)
            self.velmsg.linear.x     = linvelsetpt
            self.velmsg.angular.z    = angvelsetpt
            print('P2P mocap pub vel', self.velmsg.linear.x, self.velmsg.angular.z)
            # Publish required velocity profile (v, w)
            self.pub_vel()

        # Path planning
        elif (self.currentstep < self.numofstep - 1):
                        
            self.currentstep += 1
            
            i = self.currentstep

            next_goal = Pose2D()
            
            #shift the coordinates with 50 to fit the self-defined grid
            if (self.currentstep < self.numofstep - 1):
                next_goal.x = (self.path[i+1][0] - 50) / 10.0
                next_goal.y = (self.path[i+1][1] - 50) / 10.0

            self.goal_pose = next_goal
            print('--------------------------------------------------------')
            print('step:', self.currentstep)
            print('next goal==> x: ',self.goal_pose.x, 'y: ',self.goal_pose.y)
            #print('path: ',self.path)

        else:
            # Robot stop if linear and angular tolerances are met
            self.stop()
            print(self.prefix_robot + ' reach target pose')

        self.rate.sleep()
        
    ##########################################################
    # Local Consensus Functions
    def Consensus(self, ref_state_contr, additional_radius):
        """
            Description:
                This function gives the reference frame of agent i Zetadot_i for robot agent i to its very own position control function.
                The consensus module has a form of:
                    Zeta_i^dot  = Neighbours' influence + Reference influence
                                = (1/eta_i)[SUM j from 1 to n:[a_ij[Zeta_j^dot - K(Zeta_i - Zeta_j)]]]    --> Neighbours' influence
                                    + (1/eta_i)[a_i(n + 1)[Zeta_contr^r^dot - K(Zeta_i - Zeta_contr^r)]]  --> Reference influence
                    
                for the randomly assigned leader and:
                    Zeta_i^dot  = Neighbours' influence
                                = (1/eta_i)[SUM j from 1 to n:[a_ij[Zeta_j^dot - K(Zeta_i - Zeta_j)]]]
                
                for other agents.
                Inputs:
                    ref_state_contr  : The reference frame state
                    additional_radius: Additional radius (Time varying formation)

                Global variables:
                    agent_number    : Agent number
                    g_num_of_robot  : Total number of robots
                    g_adj_mat       : Adjacency matrix of the formation graph
                    g_zeta          : The understandings of the reference state of every agent
                    g_zeta_dot      : The understandings of rate the reference state of every agent
                    g_r_rel         : Relative distance to virtual leader/ center of formation
                
                Action:
                    P2P_mocap_noPlanning()

        """

        global g_multi_robot, g_num_of_robot, g_adj_mat, g_zeta, g_zeta_dot, g_r_rel
        # Ignore leader at this stage
        K                   = 1.0 # Constant influencing the convergence speed

        # Initialize        
        eta_i               = 0   # Number of connected neighbours for agent i
        N_Inf               = 0   # Neighbours' Influence
        R_Inf               = 0   # Reference's Influence
        agent_num           = g_multi_robot.index(self.prefix_robot[:-1]) # Corresponding agent number of the Burger
        self.agent_num      = agent_num
        self.pre_ref_state  = []
        g_zeta[agent_num,0] = ref_state_contr[0] # x
        g_zeta[agent_num,1] = ref_state_contr[1] # y
        g_zeta[agent_num,2] = ref_state_contr[2] # theta
        
        # Neighbours' Influence
        for j in range(1, g_num_of_robot + 1):
            
            self.pre_ref_state.append(0)
            delta_Zeta = g_zeta[agent_num] - g_zeta[j - 1]

            # a_ij(Zetadot_j - K(delta_Zeta))
            N_Inf += g_adj_mat[agent_num, j - 1]*(g_zeta_dot[j - 1] - K*delta_Zeta)
            eta_i += g_adj_mat[agent_num, j - 1]

            # Store agents' understanding for next computation
            self.pre_ref_state[j - 1] = g_zeta[j - 1]

        N_Inf = N_Inf / eta_i
        
        # Reference's Influence
        if g_adj_mat[agent_num, g_num_of_robot] == 1:
            delta_Zeta_c  = g_zeta[agent_num] - ref_state_contr

            # Zetadot_contr
            Zetadot_contr = (ref_state_contr - self.pre_ref_state_contr)/self.dt

            # (1 / eta_i)*a_i(n+1)*(Zetadot_contr - K(Zeta_i - Zeta_contr))
            R_Inf = (1 / eta_i)*g_adj_mat[agent_num, len(g_adj_mat[agent_num]) - 1]*(Zetadot_contr - K*delta_Zeta_c)

            self.pre_ref_state_contr = ref_state_contr

        else:
            R_Inf = 0

        # Consensus Law: Gives the velocity of the reference frame state in agent i's understanding
        g_zeta_dot[agent_num]           = N_Inf + R_Inf
        self.pre_ref_state[agent_num]   = g_zeta[agent_num]
        self.goal_pose                  = self.Desired_Absolute_Position(g_zeta[agent_num], g_r_rel[agent_num], additional_radius)

        self.P2P_mocap_noPlanning()

    def Desired_Absolute_Position(self, Zeta_i, r_rel_i, additional_radius):
        """
            Description:
                This function computes the desired absolute position for agent i
                depending on the reference state of agent i's own understanding and relative coordinates 

                Inputs:
                    Zeta_i      (Pose2D): Agent i's own understanding of the center position of the formation, in vector form [x, y, theta].T
                    r_rel_i     (Pose2D): Agent i's relative coordinates about the formation center, in vector form [x, y, theta].T

                Outputs:
                    rd_i        (Pose2D): Agent i's desired absolute position, will pass to agent i's dynamic control unit, in vector form [x, y, theta].T
        """
        R_xy = np.matrix([
                          [cos(Zeta_i[0,2]), -sin(Zeta_i[0,2]), 0],
                          [sin(Zeta_i[0,2]),  cos(Zeta_i[0,2]), 0],
                          [0,                 0,                1]
                         ])

        r_rel_i = np.matrix(r_rel_i)
        r_rel_i = r_rel_i.T
        
        if additional_radius:
            r_rel_i[0] = r_rel_i[0] + additional_radius*cos(r_rel_i[2])
            r_rel_i[1] = r_rel_i[1] + additional_radius*sin(r_rel_i[2])
        
        r_rel_i_absolute = R_xy*r_rel_i

        temp        = np.squeeze(np.asarray(Zeta_i + r_rel_i_absolute.T))
        rd_i        = Pose2D()
        rd_i.x      = temp[0]
        rd_i.y      = temp[1]
        rd_i.theta  = self.angle_bound(temp[2])

        return rd_i

class tb3_ctrl_master():
    """
        Description:
            Robot control class
    """
    def __init__(self):
        """
            Description:
                Initialize parameters and prepare data log directories

        Creates a node with name 'turtlebot3_teleop' and make sure it is a
        unique node (using anonymous = True).
        
        """

        rospy.init_node('tb3_PID')
        #rospy.init_node('odom')

        # Publisher of multiple Burgers
        self.ctrlfreq = 10
        self.dt       = 1.0/self.ctrlfreq
        self.rate     = rospy.Rate(self.ctrlfreq)

        self.goal_pose           = Pose2D()
        self.lastgoal_pose       = Pose2D()
        self.lastgoal_pose.x     = 0
        self.lastgoal_pose.y     = 0
        self.lastgoal_pose.theta = 0
        self.flag_datalog        = True # False

        if self.flag_datalog == True:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.prepare_dirdatalog(timestamp)

        # Formation Parameters
        self.formation_radius = 0.5

        global g_multi_robot, g_num_of_robot, g_feedback_pose, g_additional_radius
        
        g_multi_robot            = ['tb3_1','tb3_2','tb3_4','tb3_6']#'tb3_2','tb3_3','tb3_4','tb3_5','tb3_6']
        #g_multi_robot            = ['']# For Simulation
        g_num_of_robot           = len(g_multi_robot)
        self.turtlebots          = {}
        self.goalpose_publishers = {}     

        
        for item in g_multi_robot:
            g_feedback_pose[item] = Pose2D()


        for i in range(g_num_of_robot):
            self.goalpose_publishers[i] = rospy.Publisher(g_multi_robot[i] + '/goal_pose', Pose2D, queue_size = 10)
            self.turtlebots[i]          = TurtleBot(self.ctrlfreq, multi_robot_name = g_multi_robot[i], datalog = self.flag_datalog)

    def prepare_dirdatalog(self, timestamp):
        """
            Description:
                Create data log directories for logging

        """
        dir_curr         = os.path.dirname(__file__)
        self.dir_datalog = dir_curr[:dir_curr.rfind('/')] + '/datalog/' + timestamp
        os.mkdir(self.dir_datalog)

    def save_datalog(self):
        """
            Description:
                Save data log for every robot

        """
        for i in range(g_num_of_robot):
            self.turtlebots[i].update_dirdatalog(self.dir_datalog)
   
    def key_input(self):
        """
            Description:
                Main function for control scheme navigation

        """
        while not rospy.is_shutdown():
            print("""
                  ------------------------------------------
                  Choose the motion you to perform
                  1: Stop all robots
                  2: P2P_PControl
                  3: P2P_follow
                  4: Formation Control
                  """)
            choice = int(input("Your Choice: "))
            #choice = int(choice)

            #update global variable
            global g_multi_robot, g_num_of_robot, g_feedback_pose

            if choice == 1:
                for i in range(g_num_of_robot):
                    self.turtlebots[i].stop()

            elif choice == 2:
                self.P2P_PControl()

            elif choice == 3:
                self.P2P_follow()

            elif choice == 4:
                self.formation_ctrl()

            else:
                print('Invalid Input!')

        # If we press control + C, the node will stop.
        rospy.spin()

    def set_zerovel(self, vel_msg):
        """
            Description:
                Set and return linear and angular velocity as vel_msg

        """
        vel_msg.linear.x  = 0   # v
        vel_msg.linear.y  = 0   # --
        vel_msg.linear.z  = 0   # --
        vel_msg.angular.x = 0   # --
        vel_msg.angular.y = 0   # --
        vel_msg.angular.z = 0   # w

        return vel_msg

    def P2P_PControl(self):
        """
            Description:
                Trial function for formation
        """

        # Set goal pose from input
        goal_pose       = Pose2D()
        goal_pose.x     = input("Set your x goal    : ")
        goal_pose.y     = input("Set your y goal    : ")
        goal_pose.theta = input("Set your theta goal: ")

        for i in range(g_num_of_robot):

            # reset PID controller
            self.turtlebots[i].clear_PID_accu()

            # Publish goal pose
            self.goalpose_publishers[i].publish(goal_pose)
            
            # Formation
            goal_pose.x -= 0.5*i
            goal_pose.y -= 0.5*i

        # Logging
        if self.flag_datalog == True:
            self.save_datalog()

    def P2P_follow(self):
        """
            Description:
                Point to point, straight line, single robot control

                Update positional parameters at t - 1 (last goal pose) and t (goal pose):
                    Positional parameters at   t - 1 = t
                    Positional parameters at   t     = t + 1 (next position by manual input)

                Publish the goal pose to master PC

        """
        self.lastgoal_pose.x     = self.goal_pose.x
        self.lastgoal_pose.y     = self.goal_pose.y
        self.lastgoal_pose.theta = self.goal_pose.theta

        self.goal_pose.x     = input("Set your x goal    : ")
        self.goal_pose.y     = input("Set your y goal    : ")
        self.goal_pose.theta = input("Set your theta goal: ")

        self.goalpose_publishers[0].publish(self.goal_pose)
        self.goalpose_publishers[1].publish(self.lastgoal_pose)

    def formation_ctrl(self):
        """
            Description:
                Formation control menu
        """
        formation_ = Formation()
        global g_start_time, g_multi_robot, g_num_of_robot, g_adj_mat, g_zeta, g_zeta_dot, g_r_rel, g_ref_state_contr, g_additional_radius

        g_adj_mat, min_spanning_tree = formation_.Random_Graph_Generation(g_num_of_robot)
        print("""
              ------------------------------------------
              Choose a formation:
              1: Single Point Formation
              2: Time Varying Formation
              """)
        FormationChoice = int(input("Your Choice: "))
        if FormationChoice == 1:
            IsVaryTimeFormation = 0
        elif FormationChoice == 2:
            IsVaryTimeFormation = 1
        else:
            print('Invalid Input!')


        print("""
              ------------------------------------------
              Choose a formation:
              1: Standard polygon
              2: Line
              """)
        choice = int(input("Your Choice: "))

        x                       = input("Set Formation centre x: ")
        y                       = input("Set Formation centre y: ")
        theta                   = (pi/180)*input("Set Formation centre theta (degree): ")
        self.formation_radius   = input("Set Formation radius (m): ")
        g_ref_state_contr       = np.array([x,y,theta])

        
        if choice == 1:
            g_r_rel = formation_.Standard_Polygons_Formation(g_num_of_robot, self.formation_radius)
        elif choice == 2:
            g_r_rel = formation_.Line_Formation(g_num_of_robot, self.formation_radius)
        else:
            print('Invalid Input!')

        # Formation centre


        if IsVaryTimeFormation == 1:
            # Create Check Points
            print("""
                ------------------------------------------
                Choose a Time Varying Form:
                1: Self Rotation
                2: Flower
                3: Spiral
                """)
            VaryTimeForm = int(input("Your Choice: "))
            if VaryTimeForm == 1:
                number_of_checkpoints = int(input("Number of Checkpoints: "))
                g_ref_state_contr, g_additional_radius = formation_.Rotation(g_ref_state_contr, number_of_checkpoints, g_num_of_robot, VaryTimeForm - 1)
            elif VaryTimeForm == 2:
                number_of_checkpoints = int(input("Number of Checkpoints: "))
                g_ref_state_contr, g_additional_radius = formation_.Rotation(g_ref_state_contr, number_of_checkpoints, g_num_of_robot, VaryTimeForm - 1)
            elif VaryTimeForm == 3:
                number_of_checkpoints = int(input("Number of Checkpoints: "))
                g_ref_state_contr, g_additional_radius = formation_.Rotation(g_ref_state_contr, number_of_checkpoints, g_num_of_robot, VaryTimeForm - 1)
            else:
                print('Invalid Input!')

        g_zeta          = np.matrix(np.zeros((g_num_of_robot,3)))
        g_zeta_dot      = np.matrix(np.zeros((g_num_of_robot,3)))
        pose_temp       = Pose2D()

        if IsVaryTimeFormation == 1:
            pose_temp.x     = 1 + g_ref_state_contr[0][0]/2
            pose_temp.y     = 2 + g_ref_state_contr[0][1]/2
            pose_temp.theta = 3 + g_ref_state_contr[0][2]/2
        else:
            pose_temp.x     = 1 + g_ref_state_contr[0]/2
            pose_temp.y     = 2 + g_ref_state_contr[1]/2
            pose_temp.theta = 3 + g_ref_state_contr[2]/2

        g_start_time = time.time()  # Starting time of the formation

        for i in range(g_num_of_robot):
            self.turtlebots[i].flag_consensus = True
            self.goalpose_publishers[i].publish(pose_temp)

        # Logging
        if self.flag_datalog == True:
            self.save_datalog()

class Formation():
    """
        This class gives the formations of n agents in various shape in a matrix manner:
        r_rel = [
                [rx_1, ry_1, rtheta_1]
                [rx_2, ry_2, rtheta_2]
                [rx_3, ry_3, rtheta_3]
                        ...
                [rx_i, ry_i, rtheta_i]
                        ...
                [rx_n, ry_n, rtheta_n]
                ]
        
        Every row represents the distance between the formation center and the desired position of agent i.
    """
    def __init__(self):
        """
        Variable declaration

        """
        self.number_of_agents = 0  # Number of agents
        self.formation_radius = 1  # formation radius
        self.length           = 1  # formation length
        self.r_rel            = [] # Formation

    ##########################################################
    # Formation
    def Standard_Polygons_Formation(self, number_of_agents, formation_radius):
        """
            Description:
                This function returns the relative coordinates r_rel for n agents to form a standard polygon in a matrix manner.

                Inputs:
                    number_of_agents:   Number of agents
                    formation_radius:   Radius of formation from formation center
                
                Outputs:
                    r_rel:              Formation of standard polygons

        """
        r_rel = []
        
        if number_of_agents > 0:
            for i in range(1, number_of_agents + 1):
                r_i       = Pose2D()
                r_i.theta = 2*pi*(float(i) / number_of_agents)
                r_i.x     = formation_radius*cos(r_i.theta)
                r_i.y     = formation_radius*sin(r_i.theta)
                
                r_rel.append([r_i.x, r_i.y, r_i.theta])
        else:
            print("There is no agent.")

        #self.r_rel = r_rel

        return r_rel

    def Line_Formation(self, number_of_agents, length):
        """
            Description:
                This function returns the relative coordinates r_rel for n agents to form a standard polygon in a matrix manner.

                Inputs:
                    number_of_agents:   Number of agents
                    length:             Length of the line
                
                Outputs:
                    r_rel:              Formation of standard polygons

        """
        r_rel       = []
        orientation = (pi/180)*input("Formation along (degree): ")

        if number_of_agents > 0:
            for i in range(1, number_of_agents + 1):

                r_i       = Pose2D()
                r_i.theta = orientation
                r_i.x     = (length*i/number_of_agents) * cos(r_i.theta)
                r_i.y     = (length*i/number_of_agents) * sin(r_i.theta)
                
                r_rel.append([r_i.x, r_i.y, r_i.theta])
        
        else:
            print("There is no agent.")

        return r_rel

    ##########################################################
    # Formation Path
    def Rotation(self, g_ref_state_contr, number_of_checkpoints, number_of_robot, Varying_Radius_Mode):
        """
        Varying_Radius_Mode = 1 --> Flower: additional radius = |sin(nt)|

        ** n = number of robot
        """
        temp_g_ref_state_contr = np.zeros((number_of_checkpoints + 1, len(g_ref_state_contr)))
        additional_radius      = np.zeros((number_of_checkpoints + 1, 1))
        for i in range(0, number_of_checkpoints + 1):
            temp_g_ref_state_contr[i] = [g_ref_state_contr[0], g_ref_state_contr[1], (g_ref_state_contr[2] + 2*pi*i/number_of_checkpoints)]
            if Varying_Radius_Mode == 1:
                # Flower: additional radius = |sin(nt)|
                additional_radius[i] = abs(sin(number_of_robot*(2*pi*i/number_of_checkpoints)))
            elif Varying_Radius_Mode == 2:
                additional_radius[i] = float(i)/number_of_checkpoints
        #print(additional_radius)
        return temp_g_ref_state_contr, additional_radius

    def Circular_Path_Around_Point(self, g_ref_state_contr, number_of_checkpoints):
        temp_g_ref_state_contr = np.zeros((number_of_checkpoints, len(g_ref_state_contr)))
        Circular_Path_Radius = input("Circular Path Radius: ")
        for i in range(0, number_of_checkpoints):
            temp_g_ref_state_contr[i] = [(g_ref_state_contr[0] + Circular_Path_Radius*cos(2*pi*i/number_of_checkpoints)),
                                         (g_ref_state_contr[1] + Circular_Path_Radius*sin(2*pi*i/number_of_checkpoints)), 
                                         (g_ref_state_contr[2] + 2*pi*i/number_of_checkpoints)]

        return temp_g_ref_state_contr

    ##########################################################
    # Connection Map
    def Random_Graph_Generation(self, number_of_agents):
        """
            Description:
                This function randomly generates a graph of n agents with Virtual Leader 
                without self loops using Erdos Renyi Algorithm and find the minimum spanning tree.

                Inputs:
                    number_of_agents    : Number of agents

                Outputs:
                    Adj_mat             : Adjacency matrix with Virtual Leader
                    min_spanning_tree   : Minimum spanning tree of the generated graph

        """


        # Randomly generate a graph with given number of agents
        P         = 0.6 # Each of the possible edges with probability P (defalut = 0.6)
        print("Graph is generating with Each of the possible edges with probability ", P)
        Ran_Graph = nx.erdos_renyi_graph(number_of_agents, P)

        # We connect?
        while nx.is_connected(Ran_Graph) == False:
            Ran_Graph = nx.erdos_renyi_graph(number_of_agents, P)

        # Extract Adjacency matrix and minimim spanning tree
        Adj_mat_withoutVL = nx.adjacency_matrix(Ran_Graph).todense()
        min_spanning_tree = sorted(nx.minimum_spanning_tree(Ran_Graph).edges)

        # Random generate connections between Virtual Leader and agents
        Init_P_Zero = 0.8
        Init_P_One  = 0.2
        Change_Step = 0.01
        Rand_VLead_Connection = np.asmatrix(np.random.choice([0, 1], size = number_of_agents, p = [Init_P_Zero, Init_P_One])) # Shape = 1 x N
        while np.where(Rand_VLead_Connection == 1)[1].size == 0:
            Rand_VLead_Connection = np.asmatrix(np.random.choice([0, 1], size = number_of_agents, p = [Init_P_Zero, Init_P_One]))
            Init_P_Zero -= Change_Step
            Init_P_One  += Change_Step

        # Overall Adjacency matrix:
        Adj_mat = np.bmat([
                           [Adj_mat_withoutVL,     Rand_VLead_Connection.T],
                           [Rand_VLead_Connection, np.matrix([0])]
                          ])

        # Showcase
        nx.draw(Ran_Graph, with_labels = True)
        # plt.show()
        print("Minimum spanning tree (without Virtual Leader):")
        print(min_spanning_tree)
        
        print("It is randomly assigned the Virtual Leader to be connected with agent number(s):")
        print(np.where(Rand_VLead_Connection == 1)[1] + 1)
        print("Overall Adjacency matrix (with Virtual Leader):")
        print(Adj_mat)

        return Adj_mat, min_spanning_tree

    ##########################################################
    # Menu
    def Formation_Selection(self):
        print("""
              ------------------------------------------
              Choose a formation:
              1: Standard polygon
              2: Line
              """)
        
        choice = int(input("Your Choice: "))

        # Subject to modify
        self.number_of_agents = 2
        self.formation_radius = 1

        if choice == 1:
            print('Start Stand polygon formation control')
            self.r_rel = self.Standard_Polygons_Formation(self.number_of_agents, self.formation_radius)
        
        elif choice == 2:
            length = self.formation_radius
            self.Line_Formation(self.number_of_agents, length)

        else:
            print('Invalid Input!')

if __name__ == '__main__':
    try:
        x = tb3_ctrl_master()
        x.key_input()

    except rospy.ROSInterruptException:
        pass
