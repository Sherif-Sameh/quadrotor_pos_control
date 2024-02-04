#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu
from test_package.msg import DroneTrajectory
from test_package.msg import FullState
from mavros_msgs.msg import AttitudeTarget
from tf.transformations import euler_from_matrix, quaternion_matrix, quaternion_from_matrix, euler_matrix
'''
    This node is to capture the absolute position states of the drone and the platform in order to provide the 
    operating nodes that later utilize them to provide the controller with its inputs.
'''

####################### Global variables declarations #######################
#                                           0     1      2       3      4     5    6   7   8   9  10   11
'''These are the states of our system x = [phi phi_dot theta theta_dot psi psi_dot x x_dot y y_dot z z_dot]'''
x = np.zeros(12)

x_des = np.zeros(12)
x_des_prev = np.zeros(12)
x_des_dot = np.zeros(12)
#                                         0   1   2   3   4   5
'''X_hat contains the following: x_hat = [x x_dot y y_dot z z_dot]'''
x_hat = np.zeros(6)
x_hat_prev = np.zeros(6)

z_tilda_integral = 0
Yaw_des = 0

'''This contains the sliding surface components [Sx Sy Sz]'''
sliding_surface = np.zeros(3)

'''These will be the controller outputs'''
U_1 = 0
U_x = 0
U_y = 0

'''Output msg objects'''
attitude_target = AttitudeTarget()
state_estimate = FullState()

'''Rotation Transform (R_B)^A (Rotation matrix of frame B w.r.t. frame A)'''
RBA_matrix = 0
RAM_matrix = 0

'''Flags'''
obtained_RAM = False
obtained_RBA = False

####################### Loading parameters from ROSPARAM server #######################

rospy.init_node("smc_controller_observer")

'''Drone parameters'''
m = rospy.get_param('~mass','default_value')
g = rospy.get_param('~gravity','default_value')
max_thrust_per_motor = rospy.get_param('~max_thrust_per_motor','default_value')

'''Observer parameters'''
epsilon_x = rospy.get_param('~epsilon_x','default_value')
alpha_x1 = rospy.get_param('~alpha_x1','default_value')
alpha_x2 = rospy.get_param('~alpha_x2','default_value')

epsilon_y = rospy.get_param('~epsilon_y','default_value')
alpha_y1 = rospy.get_param('~alpha_y1','default_value')
alpha_y2 = rospy.get_param('~alpha_y2','default_value')

epsilon_z = rospy.get_param('~epsilon_z','default_value')
alpha_z1 = rospy.get_param('~alpha_z1','default_value')
alpha_z2 = rospy.get_param('~alpha_z2','default_value')

sampling_time = rospy.get_param('~sampling_time','default_value')

h1_x = alpha_x1 / epsilon_x
h2_x = alpha_x2 / (epsilon_x**2)

h1_y = alpha_y1 / epsilon_y
h2_y = alpha_y2 / (epsilon_y**2)

h1_z = alpha_z1 / epsilon_z
h2_z = alpha_z2 / (epsilon_z**2)

'''Controller parameters'''
smc_alpha = rospy.get_param('~smc_alpha','default_value') 
K1 = rospy.get_param('~K1','default_value')
Ky = rospy.get_param('~Ky','default_value')
Kx = rospy.get_param('~Kx','default_value')

max_thrust = (max_thrust_per_motor/1000.0) * 4 * g

'''Topic names'''
imu_topic_name = rospy.get_param('~imu_topic_name','default_value')
mcs_topic_name = rospy.get_param('~mcs_topic_name','default value')
mavros_attitude_target_topic_name = rospy.get_param('~mavros_attitude_target_topic_name','default_value')
trajectory_generator_topic_name = rospy.get_param('~trajectory_generator_topic_name','default_value')

def saturate(value, boundary_thickness):
    return max(min(value/boundary_thickness, 1.0), -1.0)

def rk4(x0:np.array, TS, Ux, Uy, Uz):
    # If drone is on the ground then the effect of g is compensated by a normal force
    if U_1 < (0.4*m*g):
        g_effective = 0
    else:
        g_effective = g
    # update states function
    update_states = lambda x0: np.array(
        [x0[1] + h1_x * (x[6] - x0[0]),
         U_1 * Ux / m + h2_x * (x[6] - x0[0]),
         x0[3] + h1_y * (x[8] - x0[2]),
         U_1 * Uy / m + h2_y * (x[8] - x0[2]),
         x0[5] + h1_z * (x[10] - x0[4]),
         U_1 * Uz / m - g_effective + h2_z * (x[10] - x0[4])]
         )

    # getting K1 
    k1 = update_states(x0)

    # getting K2
    x11 = x0 + (TS/2)*k1
    k2 = update_states(x11)

    # getting k3
    x12 = x0 + (TS/2)*k2
    k3 = update_states(x12)

    # getting k4
    x13 = x0 + TS*k3
    k4 = update_states(x13)

    # computing x(k+1)

    x_next = x0 + (TS/6)*(k1 + 2*k2 + 2*k3 + k4)

    return x_next #Note that python now converted x_next to be a 6x1 vector therefore h_hat will be a 6x1 vector and will need to be addressed as x_hat[i][j]

def high_gain_observer():
    global x_hat
    global x_hat_prev

    # calculating the values of Ux, Uy, and uz based on phi, theta, and psi
    cos_phi, sin_phi = np.cos(x[0]), np.sin(x[0])
    sin_theta = np.sin(x[2])
    cos_psi, sin_psi = np.cos(x[4]), np.sin(x[4])
    Ux = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi
    Uy = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi
    Uz = cos_phi * np.cos(x[2])
    
    # performing the RK4 discritization
    x_hat_prev = np.copy(x_hat)
    x_hat = rk4(x_hat, sampling_time, Ux, Uy, Uz)


def trajectory_callback(reference_states:DroneTrajectory):
    global x_des
    global x_des_prev
    global x_des_dot
    global Yaw_des
    
    Yaw_des = reference_states.yaw
    x_des_prev = np.copy(x_des)

    x_des[6] = reference_states.position.x
    x_des[7] = reference_states.velocity.x
    x_des[8] = reference_states.position.y
    x_des[9] = reference_states.velocity.y
    x_des[10] = reference_states.position.z
    x_des[11] = reference_states.velocity.z

    x_des_dot[6] = reference_states.velocity.x
    x_des_dot[7] = reference_states.acceleration.x
    x_des_dot[8] = reference_states.velocity.y
    x_des_dot[9] = reference_states.acceleration.y
    x_des_dot[10] = reference_states.velocity.z
    x_des_dot[11] = reference_states.acceleration.z


def imu_data_callback(imu_data:Imu):
    global x
    global RBA_matrix
    global obtained_RBA

    curr_orientation_in_quat = [imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w]
    RBA_matrix = quaternion_matrix(curr_orientation_in_quat)

    # This step is to extract the rotation matrix only
    RBA_matrix = RBA_matrix[:3,:3]
    obtained_RBA = True

    if obtained_RAM:
        RBM_curr = np.matmul(RAM_matrix, RBA_matrix)
        curr_orientation_in_euler = np.array(euler_from_matrix(RBM_curr, axes='sxyz'))
        # print(f"Current Orientation (AP): {curr_orientation_in_euler}")

        x[0] = curr_orientation_in_euler[0]
        x[2] = curr_orientation_in_euler[1]
        x[4] = curr_orientation_in_euler[2]

def mcs_callback(mcs_data:PoseStamped):
    global x
    global RAM_matrix
    global obtained_RAM

    x[6] = mcs_data.pose.position.x
    x[8] = mcs_data.pose.position.y
    x[10] = mcs_data.pose.position.z

    if obtained_RAM:
        controller_output()

    elif obtained_RBA:
        # Calculate rotation matrix from frame AP to frame MCS
        curr_orientation_in_quat = [mcs_data.pose.orientation.x,mcs_data.pose.orientation.y,mcs_data.pose.orientation.z,mcs_data.pose.orientation.w]
        RBM_matrix = quaternion_matrix(curr_orientation_in_quat)
        RBM_matrix = RBM_matrix[:3,:3]
        RAM_matrix = np.matmul(RBM_matrix, np.transpose(RBA_matrix))
        obtained_RAM = True

        # Report calculated values
        curr_orientation_in_euler_mcs = np.array(euler_from_matrix(RBM_matrix, axes='sxyz'))
        curr_orientation_in_euler_ap = np.array(euler_from_matrix(RBA_matrix, axes='sxyz'))
        offset_orientation_in_euler = np.array(euler_from_matrix(RAM_matrix, axes='sxyz'))
        print(f"Current Orientation (MCS): {curr_orientation_in_euler_mcs}")
        print(f"Current Orientation (AP): {curr_orientation_in_euler_ap}")
        print(f"Offset between AP and MCS: {offset_orientation_in_euler}")

def sliding_surfaces_computer():
    global sliding_surface
    global z_tilda_integral

    '''
        since for odd j (j=1:2:6) z(j) = x_des(j) - x(j)  and z(6+j) = x_des(6+j) - x_hat(j) therefore

        z(7) = x_des(7) - x_hat(1)
        z(9) = x_des(9) - x_hat(3)
        z(11) = x_des(11) - x_hat(5)

        even j (j=2:2:6) z(j) = x(j) - x_des(j) - alpha*z(j-1) and z(6+j) = x_hat(j) - x_des(6+j) - alpha*z(5+j) therefore

        Sx = z(8) = x_hat(2) - x_des(8) - alpha*z(7)
        Sy = z(10) = x_hat(4) - x_des(10) - alpha*z(9)
        Sz = z(12) = x_hat(6) - x_des(12) - alpha*z(11)

        Keeping in mind that these equations were on MATLAB which indices start from 1 instead of 0, we will have to shift
        everything by -1
    '''
    # Estimate integral of tracking errors in Z using Trapezoidal integration
    if sliding_surface[2] < 0.4:
        z_tilda_integral += (sampling_time/2) * ((x_hat_prev[4] - x_des_prev[10]) + (x_hat[4] - x_des[10]))

    sliding_surface[0] = x_hat[1] - x_des[7] + smc_alpha*(x_hat[0] - x_des[6])
    sliding_surface[1] = x_hat[3] - x_des[9] + smc_alpha*(x_hat[2] - x_des[8])
    sliding_surface[2] = x_hat[5] - x_des[11] + 2*smc_alpha*(x_hat[4] - x_des[10]) + (smc_alpha**2)*z_tilda_integral 
    # print("sliding surface done")

def controller_output():
    high_gain_observer()
    sliding_surfaces_computer()

    Cphi = np.cos(x[0])
    Ctheta = np.cos(x[2])

    '''Thrust Vector Input'''
    a = (m/(Cphi*Ctheta))
    b = -K1*saturate(sliding_surface[2], 0.4) + x_des_dot[11] - 2*smc_alpha*(x_hat[5] - x_des[11]) \
        - (smc_alpha**2)*(x_hat[4] - x_des[10]) + g
    U_1 = a*b
    
    U_1 = max(min(U_1, max_thrust),0)
    # print("Thrust vector a: {} b:{}  U_1: {}".format(a,b,U_1))

    '''Virtual Input'''
    if U_1 == 0:
        U_x = 0
        U_y = 0
    else:
        a = m/U_1
        b = -Kx*saturate(sliding_surface[0],0.5) + x_des_dot[7] - smc_alpha*(x_hat[1] - x_des[7])
        U_x = a*b
        # print("Ux vector a: {} b:{}  U_x: {}".format(a,b,U_x))

        b = -Ky*saturate(sliding_surface[1],0.5) + x_des_dot[9] - smc_alpha*(x_hat[3] - x_des[9])
        U_y = a*b
        # print("Uy vector a: {} b:{}  U_y: {}".format(a,b,U_y))

    '''Converting from U_x and U_y into attitude angles [-pi/3, pi/3]'''
    sin_max_angle = np.sin(5*np.pi/12)
    sin_phi_des = U_x*np.sin(Yaw_des) - U_y*np.cos(Yaw_des)
    phi_des = np.arcsin(max(min(sin_phi_des, sin_max_angle), -sin_max_angle))

    sin_theta_des = (U_x - np.sin(phi_des) * np.sin(Yaw_des)) / (np.cos(phi_des) * np.cos(Yaw_des))
    theta_des = np.arcsin(max(min(sin_theta_des, sin_max_angle), -sin_max_angle))
    
    '''Obtaining the desired rotation in the AP frame '''
    RBM_desired = np.array(euler_matrix(phi_des, theta_des, Yaw_des, axes='sxyz'))
    RBM_desired = RBM_desired[:3,:3]
    RBA_desired = np.matmul(np.transpose(RAM_matrix), RBM_desired)

    ''' Converting the desired orientation into a quaternion '''
    orientation_desired = np.eye(4)
    orientation_desired[:3,:3] = RBA_desired
    orientation_desired_quat = quaternion_from_matrix(orientation_desired)

    attitude_target.header.stamp = rospy.Time.now()
    attitude_target.type_mask = 7
    attitude_target.orientation.x = orientation_desired_quat[0]
    attitude_target.orientation.y = orientation_desired_quat[1]
    attitude_target.orientation.z = orientation_desired_quat[2]
    attitude_target.orientation.w = orientation_desired_quat[3]
    attitude_target.thrust = U_1/max_thrust

    '''Publish the desired thrust and attitude'''
    attitude_target_publisher.publish(attitude_target)

    '''Publish state estimate for debugging'''
    state_estimate.header.stamp = attitude_target.header.stamp
    state_estimate.pose.position.x  = x_hat[0]
    state_estimate.twist.linear.x   = x_hat[1]
    state_estimate.pose.position.y  = x_hat[2]
    state_estimate.twist.linear.y   = x_hat[3]
    state_estimate.pose.position.z  = x_hat[4]
    state_estimate.twist.linear.z   = x_hat[5]
    state_estimate_publisher.publish(state_estimate)

if __name__ == '__main__':
    
    ####################### Initiating node and obtaining parameters #######################

    # rospy.init_node("model_positions_listener")
    rospy.loginfo("states listener launched")

    ####################### Topics declarations #######################

    imu_topic = (imu_topic_name,Imu)
    mcs_topic = (mcs_topic_name,PoseStamped)
    trajectory_topic = (trajectory_generator_topic_name,DroneTrajectory)
    mavros_attitude_target_topic = (mavros_attitude_target_topic_name, AttitudeTarget)
    state_estimate_topic = ('/iris_drone/state_estimate', FullState)

    # ####################### Node subscriptions #######################
    
    imu_data_subscriber = rospy.Subscriber(imu_topic[0],imu_topic[1],imu_data_callback)
    trajectory_subscriber = rospy.Subscriber(trajectory_topic[0],trajectory_topic[1],trajectory_callback)
    mcs_subscriber = rospy.Subscriber(mcs_topic[0],mcs_topic[1],mcs_callback)

    ####################### Node publications #######################

    attitude_target_publisher = rospy.Publisher(mavros_attitude_target_topic[0],mavros_attitude_target_topic[1],queue_size=1)
    state_estimate_publisher = rospy.Publisher(state_estimate_topic[0], state_estimate_topic[1], queue_size=1)

    rospy.spin()