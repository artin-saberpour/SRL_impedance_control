

#import controller.robot_6dof_newDH_fixing as r
from types import SimpleNamespace
import math
from scipy.spatial.transform import Rotation


import mujoco
import mujoco.viewer as viewer
import numpy as np
import time
import threading
import hw_interface

from collections import deque
import matplotlib.pyplot as plt

args = SimpleNamespace(
    motor_ids = [1, 2, 3, 4, 5, 6],  # IDs of the motors you want to control
    goal_velocities = [0, 0, 0, 0, 0, 0],  # Velocities in Dynamixel units (adjust according to your motor's range)
    motor_offsets = [0, 1024, 2048, 3072, 2048, 2048],
    ADDR_PRO_GOAL_VELOCITY = 104, # This can vary depending on the Dynamixel model
    ADDR_PRESENT_POSITION = 132,
    ADDR_TORQUE_ENABLE = 64,
    ADDR_OPERATING_MODE = 11,        # Typically the address for the Operating Mode
    ADDR_GOAL_POSITION = 116,
    LEN_PRO_GOAL_VELOCITY = 4,     # Length of the goal velocity value (4 bytes for a 32-bit value)
    LEN_PRO_PRESENT_POSITION = 4,
    PROTOCOL_VERSION = 2.0,
    POSITION_CONTROL_MODE = 3,
    VELOCITY_CONTROL_MODE = 1,
    TORQUE_CONTROL_MODE = 0,
    ADDR_GOAL_CURRENT = 102,
    LEN_PRO_GOAL_CURRENT = 2,
    BAUDRATE = 57600,#1000000,  # Adjust based on your Dynamixel motor configuration
    DEVICENAME = '/dev/tty.usbserial-FT7928RB',#'/dev/tty.usbserial-FT7W92VO',#'/dev/tty.usbserial-FT7W93U3',  # Replace with the actual port
    hw_interface = 'defualt',
    velocity_controller = 'default',
    q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    links = np.array([20.0, 20.0, 20.0, 5.0, 3.5, 5.0]),
    motor_steps = 4096,
    velocity_limit = 50,#1023,
    motorStep_to_radians = 0.0,
    robot_dof=6
    )
args.motorStep_to_radians = 2 * math.pi / args.motor_steps




# Function to compute task-space inertia matrix (Lambda)
def compute_task_space_inertia(H, J):
    return np.linalg.inv(J @ np.linalg.inv(H) @ J.T)


# Impedance control function
def impedance_control(dx, dx_des, M_d, K_p, K_d, dt=1):#(x, x_des, dx, dx_des, M_d, K_p, K_d, dt=1e-3):

    return M_d @ (K_p * dx_des/dt - K_d * dx)



def has_bad_values(arr, threshold=1e3):
    if np.any(np.isnan(arr)) or np.any(np.isinf(arr)) or np.any(np.abs(arr) > threshold):
        print(arr)
        print("********************")
    return np.any(np.isnan(arr)) or np.any(np.isinf(arr)) or np.any(np.abs(arr) > threshold)



def get_current_ee_pos(model, data, end_effector_id):
    pos = data.site_xpos[end_effector_id]
    
    # Convert 3×3 rotation matrix to Euler angles
    rot_matrix = data.site_xmat[end_effector_id].reshape(3, 3)
    euler_xyz = Rotation.from_matrix(rot_matrix).as_euler('xyz', degrees=False)  # Change 'xyz' to preferred order

    #return pos, euler_xyz  # Return both position and Euler angles
    return np.concatenate([pos, euler_xyz])

def numerical_jacobian(model, data, end_effector_id, delta=1e-8):
    nq = model.nv  # Number of velocity DoFs
    J_num = np.zeros((6, nq))  # Full spatial Jacobian

    # Save original qvel
    qpos_original = np.copy(data.qpos)
    xpos_original = np.copy(get_current_ee_pos(model, data, end_effector_id))
    
    #cvel_original = np.copy(data.cvel)
    for i in range(nq):
        # Perturb joint velocity i
        data.qpos[i] += delta
        mujoco.mj_forward(model, data)  # Update FK

        # Compute new velocity of end-effector
        dx_new = get_current_ee_pos(model, data, end_effector_id)# np.copy(data.cvel[end_effector_id])  # Full 6D velocity

        print("orig ee: ", xpos_original)
        print("new ee:  ", dx_new)
        
        # Compute numerical Jacobian column-wise
        J_num[:, i] = (dx_new - xpos_original) / delta

        # Reset velocity
        data.qpos[:] = qpos_original
        mujoco.mj_forward(model, data)
    #print("J_num shape is ------->", J_num.shape)
    return J_num


ctrl_history = deque(maxlen=3000)
time_history = deque(maxlen=3000)
plot_wake_up = False


def plot_ctrl_live():
    plt.ion()
    fig, ax = plt.subplots()
    lines = [ax.plot([], [], label=f'joint {i+1} torque')[0] for i in range(args.robot_dof)]
    ax.set_ylim(-1, 1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (Nm)")
    ax.legend()

    window_duration = 3.0  # seconds to display

    while True:
        plot_update_event.wait()
        plot_update_event.clear()

        if len(time_history) > 1:
            times = np.array(time_history)
            torques = np.array(ctrl_history)

            # Filter only the last `window_duration` seconds
            current_time = times[-1]
            valid_indices = times >= (current_time - window_duration)
            times_window = times[valid_indices]
            torques_window = torques[valid_indices]

            for i, line in enumerate(lines):
                line.set_xdata(times_window)
                line.set_ydata(torques_window[:, i])
            ax.set_xlim(times_window[0], times_window[-1])
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.0001)

            
plot_thread = threading.Thread(target=plot_ctrl_live, daemon=True)
plot_thread.start()
plot_update_event = threading.Event()


        
        
# Load the model from the XML string
model = mujoco.MjModel.from_xml_path("srl.xml")#("robot_moving_base.xml")
data = mujoco.MjData(model)





c=1
model.body_mass *=c
model.body_inertia *= c
# Assuming `model` is already created using mujoco.MjModel.from_xml_string(xml)
for i in range(model.nbody):  # Loop over all bodies
    # Retrieve body name using mj_id2name
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    mass = model.body_mass[i]
    inertia = model.body_inertia[i]
    print(f"Body: {body_name}, Mass: {mass}, Inertia: {inertia}")


# Remove gravity
model.opt.gravity = (0, 0, -9.81)


dt = 1
model.opt.timestep = 0.001

# Disable all collisions
model.geom_contype[:] = 0
model.geom_conaffinity[:] = 0

k = 10
print(data.qpos.shape)

'''
# INSTANTIATE THE ROBOTIC HARDWARE AND ACTIVATE THE CURRENT MODE
args.hw_interface = hw_interface.hw_interface(args)
print("p1")
args.hw_interface.enable_torque()
print("p2")
args.hw_interface.set_velocity_sync(args.goal_velocities)
print("p3")
input()
'''

data.qpos = ([np.pi/k, np.pi/k, np.pi/k, np.pi/k, np.pi/k, np.pi/k])# ([np.pi/6, np.pi/6, np.pi/6, np.pi/6, np.pi/6, np.pi/6])

real_time_factor = 1.0
next_time_step = time.time()+model.opt.timestep
sim_start = time.time()
# Target position & orientation (quaternion)
x_desired = np.array([0.4, 0, 0.2, 0, 0, 0])
#target = np.array([0.4, 0, 0.2])# np.array([0.2, 0, 0.2])  # Example target position
target_euler = np.array([0, 0, 0])  # Example target quaternion (w, x, y, z)
target_quat = Rotation.from_euler('xyz', target_euler).as_quat()

base_name = "link0"# "moving_base"# "base_motion"
base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, base_name)
print(data.xpos[base_id])
print(data.xquat[base_id])
# input()

# Get end-effector ID using the new MuJoCo API
ee_name = "ee_site"# "dof6"  # Change to your actual end-effector name
end_effector_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee_name)
print(end_effector_id)
# raise Exception("p0")


# Jacobian storage
J_pos = np.zeros((3, model.nv), dtype=np.float64)
J_rot = np.zeros((3, model.nv), dtype=np.float64)

prev_velocity_error = 0
i = 0
xpos_desired = np.array([0.2, 0.2, 0.2])#data.site_xpos[end_effector_id]
xquat_desired = np.zeros(4) 
mujoco.mju_axisAngle2Quat(xquat_desired, np.array([0, 0, 1]), 0.0)


dx_desired = np.zeros(6)
dx_new= np.zeros(6)
data.qvel[:] = 0
data.qacc[:] = 0
tau_rne = np.zeros(model.nv)

site_quat = np.zeros(4)
site_quat_conj = np.zeros(4)
error_quat = np.zeros(4)
M_inv = np.zeros((model.nv, model.nv))
Mx = np.zeros((6, 6))
m_k = 225
m_d = 2 * np.sqrt(m_k).item()
i= 22500
j = 0
k = 0
sim_time = 0.0
with mujoco.viewer.launch_passive(model, data,show_left_ui=False, show_right_ui=False) as viewer:
    viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
    while True:
        while True:
            
            xpos_desired = ([0.1, 0.1, 0.1])
            base_pos = data.xpos[base_id]
            xpos_desired = base_pos + np.array([0.2, 0.25 *np.sin((i/30000)*2*np.pi).item(), 0.2])
            

            pulse_period = 400  # Total period of the pulse
            pulse_duration = 80  # Duration of the pulse (must be < pulse_period)
            
            if j % pulse_period < pulse_duration:
                if j % pulse_period ==0:
                    k+=1
                if k%3==0:
                    data.mocap_pos[0] = ([0.1, 0.0, 0.0])  # Quick jump along z-axis
                elif k%3==1:
                    data.mocap_pos[0] = ([0.0, 0.1, 0.0])  # Quick jump along z-axis
                else:
                    data.mocap_pos[0] = ([0.0, 0.0, 0.1])  # Quick jump along z-axis
            else:
                data.mocap_pos[0] = ([0.0, 0.0, 0.0])  # Stay still
            
            data.mocap_quat[0] = ([1, 0, 0, 0]) # q_mj
            
            mujoco.mj_forward(model, data)  # Compute FK
            
            
            
            
            # If gravity dynamics is required, set the vel ad acc to zero and read the tau_rne
            # mujoco.mj_rne(model, data, 0, tau_rne)

            # Spatial velocity (aka twist).
            x_actual = data.site_xpos[end_effector_id]
            dx_desired[:3] = xpos_desired - x_actual# Kpos * dx / integration_dt
            mujoco.mju_mat2Quat(site_quat, data.site_xmat[end_effector_id])
            mujoco.mju_negQuat(site_quat_conj, site_quat)
            mujoco.mju_mulQuat(error_quat, xquat_desired, site_quat_conj)
            mujoco.mju_quat2Vel(dx_desired[3:], error_quat, 1.0)
            

            mujoco.mj_jacSite(model, data, J_pos, J_rot, end_effector_id)

            J = np.vstack([J_pos, J_rot])  # Full spatial Jacobian

            dx_actual = J @ data.qvel
            
            # Compute the task-space inertia matrix.
            mujoco.mj_solveM(model, data, M_inv, np.eye(model.nv))
            #print(data.qfrc_bias)
            #print("==================")
            
            MM= np.zeros((model.nv, model.nv))
            # Gives the inertia matrix
            mujoco.mj_fullM(model, MM, data.qM)

            Mx_inv = J @ M_inv @ J.T
            if abs(np.linalg.det(Mx_inv)) >= 1e-2:
                Mx = np.linalg.inv(Mx_inv)
            else:
                Mx = np.linalg.pinv(Mx_inv, rcond=1e-2)

            if np.linalg.norm(x_actual - xpos_desired) < 0.005:
                #dx_desired = np.zeros(6)
                print(x_actual)
                print(xpos_desired)
                print("model converged")
                print("--------------------")
            print("x_actual: ", x_actual)
            print("base_pos: ", base_pos)
            F  = impedance_control(dx_actual, dx_desired, Mx, np.ones(6) * m_k, np.ones(6) * m_d)

            tau = J.T @ F  # Now F has matching dimensions

            gravity_torques = np.copy(data.qfrc_bias)
            data.ctrl[:] =  gravity_torques + tau
            ctrl_history.append(np.copy(data.ctrl[:6]))  # Or [:6] if you want all 6 joints
            #time_history.append(sim_time)
            time_history.append(data.time)
            sim_time += model.opt.timestep

            #time_history.append(time.time() - sim_start)


            if (False):
                print("====================================================================")
                print(dx_old, "---", dx_actual)
                print("⚠️ Detected bad values:")
                print("x_actual:", x_actual)
                print("x_desired:", x_desired)
                print("dx_actual:", dx_actual)
                print("F:", F)
                print("tau:", tau)
                print("qpos:", data.qpos)
                print("qvel:", data.qvel)
                print(data.qfrc_inverse)
            print(data.qfrc_bias)
            print(data.qfrc_applied)
            print(data.qfrc_passive)
            print(data.qfrc_constraint)
            #input("==============")
            mujoco.mj_inverse(model, data)
            mujoco.mj_step(model, data)

            viewer.sync()
            plot_update_event.set()  # Notify the plotting thread to update
            i+=10
            j+=1
            time_to_sleep = max(next_time_step - time.time(), 0)
            # time_to_sleep = (next_time_step - time.time()) if (next_time_step - time.time() > 0) else 0
            time.sleep(time_to_sleep)
            next_time_step += model.opt.timestep
            
            # input()



