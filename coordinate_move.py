import sys
sys.path.append('..')
import logging

import rtde as rtde
import rtde_config as rtde_config

ROBOT_HOST = '10.0.0.150'
ROBOT_PORT = 30004
config_filename = 'control_loop_configuration.xml'
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

setp.input_double_register_0 = 0.0
setp.input_double_register_1 = 0.0
setp.input_double_register_2 = 0.0
setp.input_double_register_3 = 0.0
setp.input_double_register_4 = 0.0
setp.input_double_register_5 = 0.0
watchdog.input_int_register_0 = 0
setp.input_bit_registers0_to_31 = 0
final_pose = [-0.18507570121045797, -0.43755157063468963, 0.21101969081827837, -0.06998478570599498, -3.0949971695297402, 0.10056260631290592]
def setp_to_list(setp):
    list = []
    for i in range(0,6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

def list_to_setp(setp, list):
    for i in range (0,6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

if not con.send_start():
    sys.exit()

while True:
    # receive the current state
    state = con.receive()
    
    if state is None:
        break
    
    # do something...
    if state.output_int_register_0 != 0:
        print(state.actual_TCP_pose)
        list_to_setp(setp, final_pose)
        # send new setpoint        
        con.send(setp)

    # kick watchdog
    con.send(watchdog)

con.send_pause()

con.disconnect()
