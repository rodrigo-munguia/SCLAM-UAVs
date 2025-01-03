# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 Mavic 2 Pro driver."""

import math
import rclpy
import os
from geometry_msgs.msg import Twist
from interfaces.srv import SimpleServ
from interfaces.msg import Robotstate
import time


#K_VERTICAL_THRUST = 68.5    # with this thrust, the drone lifts.
K_VERTICAL_THRUST = 0.0 
K_VERTICAL_P = 1.8          # P constant of the vertical PID.
K_ROLL_P = 50.0             # P constant of the roll PID.
K_PITCH_P = 30.0            # P constant of the pitch PID.
K_YAW_P = 2.0
K_X_VELOCITY_P = 1
K_Y_VELOCITY_P = 1
K_X_VELOCITY_I = 0.01
K_Y_VELOCITY_I = 0.01
LIFT_HEIGHT = 1


class Logger:
    def __init__(self, filename):
        self.filename = filename
        self.start_time = time.time()
        # Open the file in append mode
        with open(self.filename, 'w') as file:
            # Write the header row
            file.write('time,velX_d,velX,velY_d,velY,z_d,z,velYaw_d,velYaw\n')
    
    def log(self, *variables):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Format elapsed time to ss.cc
        ss = int(elapsed_time)
        cc = int((elapsed_time - ss) * 100)
        label_time = f'{ss:02}.{cc:02}'
        
         # Format numeric variables to six decimal places
        formatted_variables = [f'{float(var):.6f}' for var in variables]
        
        # Convert formatted variables to a comma-separated string
        variables_str = ','.join(formatted_variables)
        
        # Prepare the log line
        log_line = f'{label_time},{variables_str}\n'
        
        # Write the log line to the file
        with open(self.filename, 'a') as file:
            file.write(log_line)

class Logger_h:
    def __init__(self, filename):
        self.filename = filename
        self.start_time = time.time()
        # Check if the file exists
        if not os.path.exists(self.filename):
            # Create the file if it does not exist
            # Open the file in append mode
            with open(self.filename, 'w') as file:
                # Write the header row
                file.write('time,error\n')
    
    def log(self, *variables):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Format elapsed time to ss.cc
        ss = int(elapsed_time)
        cc = int((elapsed_time - ss) * 100)
        label_time = f'{ss:02}.{cc:02}'
        
         # Format numeric variables to six decimal places
        formatted_variables = [f'{float(var):.6f}' for var in variables]
        
        # Convert formatted variables to a comma-separated string
        variables_str = ','.join(formatted_variables)
        
        # Prepare the log line
        log_line = f'{label_time},{variables_str}\n'
        
        # Write the log line to the file
        with open(self.filename, 'a') as file:
            file.write(log_line)


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class MavicDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Sensors
        self.__gps = self.__robot.getDevice('gps')
        self.__gyro = self.__robot.getDevice('gyro')
        self.__imu = self.__robot.getDevice('inertial unit')
        self.__camera = self.__robot.getDevice('camera')
        #self.__range = self.__robot.getDevice('ps0')
        
        distance_sensor = self.__robot.getDevice("ds0")
        distance_sensor.enable(1000)  # Enable the distance sensor with a sampling period of 10 milliseconds
        
        print(distance_sensor)

        # Propellers
        self.__propellers = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        # State
        self.__target_twist = Twist()
        self.__vertical_ref = LIFT_HEIGHT
        self.__linear_x_integral = 0
        self.__linear_y_integral = 0
        self.__SLAM_robot_state = Robotstate()

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('mavic_driver')
        
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        
        self.__node.create_subscription(Robotstate, 'Robotstate_topic', self.__slam_robot_state_callback, 1)
        
        self.__node.create_service(SimpleServ, "webot_service", self.__service_callback)
    
        self.__land = False
        
        # log file
        self.logger_lc = Logger('logfile_lc.csv')
        self.logger_eh = Logger_h('logfile_eh.csv')
        
    def __service_callback(self, request, response):
        global K_VERTICAL_THRUST
        cmd = request.cmd
        self.__node.get_logger().info('Incoming request\n cmd: %d' % (cmd) )
        if cmd == 116:  # 't'
            self.__node.get_logger().info("Take off..")
            self.__land = False
            K_VERTICAL_THRUST = 68.5
        if cmd == 108:  # 'l'
            self.__node.get_logger().info("Landing..")
            #K_VERTICAL_THRUST = 0.0  
            self.__land = True
            error_to_home = math.sqrt(self.gps_x**2 + self.gps_y**2)
            if error_to_home < 1.0: 
                self.logger_eh.log(error_to_home)
            
        response.response = True
        return response
        
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
        
    def __slam_robot_state_callback(self,Robotstate):
        self.__SLAM_robot_state = Robotstate

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        roll_ref = 0
        pitch_ref = 0

        # Read sensors
        roll, pitch, _ = self.__imu.getRollPitchYaw()
        self.gps_x, self.gps_y, vertical = self.__gps.getValues() # for emulating robot barometer
        roll_velocity, pitch_velocity, twist_yaw = self.__gyro.getValues()
        
        vel_gps = self.__gps.getSpeed() # for emulating robot's optical flow sensor 
        vel_linear = self.__SLAM_robot_state.vels.linear
        vel_slam = math.sqrt(vel_linear.x**2 + vel_linear.y**2) 
        
        # the webot robot does not have an optical flow sensor
        # so it must be emulated from gps velocity
        velocity = vel_gps
                
        #print("gps_vel: {}, slam_vel: {}".format(vel_gps, vel_slam ))
        
        if math.isnan(velocity):
            return
         

        # Allow high level control once the drone is lifted
        if vertical > 0.2:
            # Calculate velocity
            velocity_x = (pitch / (abs(roll) + abs(pitch))) * velocity
            velocity_y = - (roll / (abs(roll) + abs(pitch))) * velocity

            # High level controller (linear velocity)
            linear_y_error = self.__target_twist.linear.y - velocity_y
            linear_x_error = self.__target_twist.linear.x - velocity_x
            self.__linear_x_integral += linear_x_error
            self.__linear_y_integral += linear_y_error
            roll_ref = K_Y_VELOCITY_P * linear_y_error + K_Y_VELOCITY_I * self.__linear_y_integral
            pitch_ref = - K_X_VELOCITY_P * linear_x_error - K_X_VELOCITY_I * self.__linear_x_integral
            
            global K_VERTICAL_THRUST
            if self.__land  == False:
                self.__vertical_ref = clamp(
                    self.__vertical_ref + self.__target_twist.linear.z * (self.__timestep / 1000),
                    max(vertical - 0.5, LIFT_HEIGHT),
                    vertical + 0.5
                )
            else:
                self.__vertical_ref = clamp(
                    self.__vertical_ref + self.__target_twist.linear.z * (self.__timestep / 1000),
                    max(vertical - 0.5, .15),
                    vertical + 0.15
                )
                            
        
        
        vertical_input = K_VERTICAL_P * (self.__vertical_ref - vertical)
        
        #print(vertical)
        

        # Low level controller (roll, pitch, yaw)
        yaw_ref = self.__target_twist.angular.z

        roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_velocity + roll_ref
        pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_velocity + pitch_ref
        yaw_input = K_YAW_P * (yaw_ref - twist_yaw)

        m1 = K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
        m2 = K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
        m3 = K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input
        m4 = K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
        
        if self.__land == True and vertical < .2:
            m1 = 0
            m2 = 0
            m3 = 0
            m4 = 0
            #print("gps x: {}, gps y: {}".format(self.gps_x, self.gps_y ))
            
        
        # Apply control
        self.__propellers[0].setVelocity(-m1)
        self.__propellers[1].setVelocity(m2)
        self.__propellers[2].setVelocity(m3)
        self.__propellers[3].setVelocity(-m4)
        
        # log out
        if self.__land == False and vertical > 1:
            self.logger_lc.log(self.__target_twist.linear.x,velocity_x,self.__target_twist.linear.y,velocity_y,self.__vertical_ref, vertical,yaw_ref, twist_yaw)