# shit to do:
"""
    1. move joints robot
        1.1 make robot move to initial point at th beginning (done)
        1.2 ask user to start the python script (done)
        1.3 check with different target joint angles1 (done)
        1.4 set up servoj example (done)
    2. change orientation
        2.1 rewatch videos (done)
        2.2 make it work once (done)
        2.3 check inverse kinematics (done)
        2.4 make it work with speedj for better gain control
            pseudocode:

            joints_desired = inverse_kin(tcp_pose_desired)
            error = joints_desired - state.actual_joints
            control effort = gain * error
            speedj(control_effort)
"""