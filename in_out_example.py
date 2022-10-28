import sys
sys.path.append('..')
import logging

import rtde as rtde
import rtde_config as rtde_config

import time
from matplotlib import pyplot as plt
from min_jerk_planner_translation import PathPlanTranslation

def setp_to_list(setp):
    list = []
    for i in range(0,6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

def list_to_setp(setp, list):
    for i in range (0,6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp
#logging.basicConfig(level=logging.INFO)

ROBOT_HOST = '10.0.0.150'
ROBOT_PORT = 30004
config_filename = 'control_loop_configuration.xml'

#keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')
setp_names, setp_types = conf.get_recipe('setp')
watchdog_names, watchdog_types = conf.get_recipe('watchdog')

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

con.get_controller_version()

con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

# Setpoints to move the robot to
setp1 = [-0.12, -0.43, 0.14, 0, 3.11, 0.04]
setp2 = [-0.12, -0.51, 0.21, 0, 3.11, 0.04]

setp.input_double_register_0 = 0.0
setp.input_double_register_1 = 0.0
setp.input_double_register_2 = 0.0
setp.input_double_register_3 = 0.0
setp.input_double_register_4 = 0.0
setp.input_double_register_5 = 0.0
  
setp.input_bit_registers0_to_31 = 0

watchdog.input_int_register_0 = 0

if not con.send_start():
    sys.exit()
state = con.receive()
start_pose = [-0.18507570121045797, -0.43755157063468963, 0.21101969081827837, -0.06998478570599498, -3.0949971695297402, 0.10056260631290592]
desired_pose = state.actual_TCP_pose
orientation_const = start_pose[3:]
tcp1 = state.actual_TCP_pose
print(tcp1)

while True:
    print('Boolean 1 is False, please click CONTINUE on the Polyscope')
    state = con.receive()
    con.send(watchdog)
    # print(f"runtime state is {state.runtime_state}")
    if state.output_bit_registers0_to_31 == True:
        print('Boolean 1 is True, Robot Program can proceed to mode 1\n')
        break

print("-------Executing moveJ -----------\n")


watchdog.input_int_register_0 = 1
con.send(watchdog)  # sending mode == 1
list_to_setp(setp, start_pose)  # changing initial pose to setp
con.send(setp) # sending initial pose

while True:
    print('Waiting for movej() to finish')
    state = con.receive()
    con.send(watchdog)
    if state.output_bit_registers0_to_31 == False:
        print('Proceeding to mode 2\n')
        break

print("-------Executing servoJ  -----------\n")
watchdog.input_int_register_0 = 2
con.send(watchdog)  # sending mode == 2

trajectory_time = 8  # time of min_jerk trajectory
dt = 1/500  # 500 Hz    # frequency
plotter = True

# ------------------ Control loop initialization -------------------------

planner = PathPlanTranslation(start_pose, desired_pose, trajectory_time)

# ----------- minimum jerk preparation -----------------------


if plotter:
    time_plot = []

    min_jerk_x = []
    min_jerk_y = []
    min_jerk_z = []

    min_jerk_vx = []
    min_jerk_vy = []
    min_jerk_vz = []

    px = []
    py = []
    pz = []

    vx = []
    vy = []
    vz = []

#   -------------------------Control loop --------------------
state = con.receive()
tcp = state.actual_TCP_pose
t_current = 0
t_start = time.time()

while time.time() - t_start < trajectory_time:
    t_init = time.time()
    state = con.receive()
    t_prev = t_current
    t_current = time.time() - t_start

    print(f"dt:{t_current-t_prev}")
    # read state from the robot
    if state.runtime_state > 1:
        #   ----------- minimum_jerk trajectory --------------
        if t_current <= trajectory_time:
            [position_ref, lin_vel_ref, acceleration_ref] = planner.trajectory_planning(t_current)

        # ------------------ impedance -----------------------
        current_pose = state.actual_TCP_pose
        current_speed = state.actual_TCP_speed

        pose = position_ref.tolist() + orientation_const

        list_to_setp(setp, pose)
        con.send(setp)

        if plotter:
            time_plot.append(time.time() - t_start)

            min_jerk_x.append(position_ref[0])
            min_jerk_y.append(position_ref[1])
            min_jerk_z.append(position_ref[2])

            min_jerk_vx.append(lin_vel_ref[0])
            min_jerk_vy.append(lin_vel_ref[1])
            min_jerk_vz.append(lin_vel_ref[2])

            px.append(current_pose[0])
            py.append(current_pose[1])
            pz.append(current_pose[2])

            vx.append(current_speed[0])
            vy.append(current_speed[1])
            vz.append(current_speed[2])

print(f"It took {time.time()-t_start}s to execute the servoJ")
print(f"time needed for min_jerk {trajectory_time}\n")

state = con.receive()
print('--------------------\n')
print(state.actual_TCP_pose)

# ====================mode 3===================
watchdog.input_int_register_0 = 3
con.send(watchdog)


con.send_pause()
con.disconnect()
