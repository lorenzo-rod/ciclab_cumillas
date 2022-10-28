#!/usr/bin/env python
# Copyright (c) 2016, Universal Robots A/S,
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Universal Robots A/S nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF 
# aVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
sys.path.append('..')   
import logging
import time
import rtde as rtde
import rtde_config as rtde_config
import keyboard
from math import sqrt

def compute_error(target, joints):
    """Computes a 6D vector containing the error in every joint in the control loop

    Args:
        target (list): List of floats containing the target joint angles in radians
        joints (list): List of floats containing the measured joint angles in radians

    Returns:
        list: List of floats containing the angles error in radians
    """
    return [j - joints[i] for i,j in enumerate(target)]

def compute_control_effort(error, gain):
    """Computes a 6D vector containing the control effort in every joint in the control loop

    Args:
        error (list): List of floats containing the angles error in radians
        gain (float): Gain in the control loop (each joint angle error will be multiplied times this value to compute control effort)

    Returns:
        liat: List of floats containing the control efforts in each joint
    """
    return [i*gain for i in error]

def list_to_degrees(angles):
    """Converts input list values from radians to degrees

    Args:
        angles (list): List containing angles in radians

    Returns:
        list: List containing angles in degrees
    """
    return [i*360/(2*3.14592) for i in angles]


ROBOT_HOST = '10.0.0.150' # ip in settings in the tablet
ROBOT_PORT = 30004
config_filename = 'control_loop_configuration.xml'

logging.getLogger().setLevel(logging.INFO)

#configuration files
conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')
setp_names, setp_types = conf.get_recipe('setp')
watchdog_names, watchdog_types = conf.get_recipe('watchdog')

#connection to the robot
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()
connection_state = con.connect()

#check connection
while connection_state != 0:
    print(connection_state)
    time.sleep(5)
    connection_state = con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

# Set initial target joint angles
target_joints = [0, -1.6, 0, -1.6, 0, 0]

# Initialize 6 registers which will hold the target angle values
setp.input_double_register_0 = 0.0
setp.input_double_register_1 = 0.0
setp.input_double_register_2 = 0.0
setp.input_double_register_3 = 0.0
setp.input_double_register_4 = 0.0
setp.input_double_register_5 = 0.0
  
# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0

# Reformat setpoint into list
def setp_to_list(setp):
    list = []
    for i in range(0,6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

# Reformat list into setpoint
def list_to_setp(setp, list):
    for i in range (0,6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

# Start data synchronization
if not con.send_start():
    sys.exit()


# Save initial time
init_time = time.time()
# Control loop, keeps running until user quits using q
while not(keyboard.is_pressed("q")):

    # Receive UR Robot state
    state = con.receive()

    # If state is None break loop and end connection
    if state is None:
        break
    
    # Read joint angles from registers
    joint_angles = state.actual_q
    # Check if the program is running in the Polyscope
    if state.output_int_register_0 != 0:
        
        # Print joint angles in degrees every second
        if(time.time() - init_time) >= 1:
            init_time = time.time()
        # Compute new target joint angles
        # ---
        # Reformat target_joints list into setpoint
        list_to_setp(setp, target_joints)
        # Send new target joints       
        con.send(setp)
        
    # kick watchdog
    con.send(watchdog)

# set target joint position to actual position
list_to_setp(setp, state.actual_q)
con.send(setp)

con.send_pause()

con.disconnect()
