#! /usr/bin/env python 
#
# diana_gripper.py
#
# Basic Python ROS node to control the 2-fingered "diana_gripper"
# built from four Feetech SCS series servos.
#
# Receives joint_states and other 
# data from the servos and forwards user-commands to the servos.
# Also publishes a sensor_msgs/Joy message with raw sensor data,
# including temperature/voltage/current for the servos.
#
# Provides (text/telnet) style commands on service ~command,
# and accepts joint position/effort goals on topic ~joint_goal.
# In addition, basic parallel-gripper operation is triggered
# by sending a float (radians) to ~simple_goal.
#
# 2022.07.04 - copy from "qbsc_gripper.py" 
# 2021.01.17 - created (based on screwdriver.py)
#
# (c) 2021, 2022 fnh, hendrich@informatik.uni-hamburg.de
#

import collections
import socket
import serial
import math
import numpy as np
import threading

import rospy
import std_msgs.msg
import sensor_msgs.msg
import control_msgs.msg 
import diana_gripper.srv

# from diana_gripper.srv import TextCommand


# global stuff
#
verbose      = 1
have_text_command = 0
text_command = "?" # help

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


# left finger / right finger, J2 = proximal J1 = distal  (same as Shadow hand)
#
hand_name    = 'diana_gripper'
n_joints = 4
short_joint_names  = ['LFJ2', 'LFJ1', 'RFJ2', 'RFJ1' ]  
joint_names  = [hand_name + '/' + s for s in short_joint_names]
joint_index = {}
for j in range( len(short_joint_names) ):
     joint_index[ short_joint_names[j]] = j
for j in range( len(joint_names) ):
     joint_index[ joint_names[j]] = j
print( joint_index ) # debugging only


# JointState and raw sensor Joy messages
#
js           = sensor_msgs.msg.JointState()  # position + velocities + torques (radians, rad/s, Nm)
js.name      = joint_names
js.position  = np.zeros( n_joints )
js.velocity  = np.zeros( n_joints )
js.effort    = np.zeros( n_joints )

js_errors    = sensor_msgs.msg.JointState()  # current pos + current vel + pos error in one message
js_errors.name      = joint_names;
js_errors.position  = np.zeros( n_joints )
js_errors.velocity  = np.zeros( n_joints )
js_errors.effort    = np.zeros( n_joints )

torques_joy  = sensor_msgs.msg.Joy()         # raw servo torques (counts)
temp_joy     = sensor_msgs.msg.Joy()         # temperatures
volt_joy     = sensor_msgs.msg.Joy()         # voltages 
curr_joy     = sensor_msgs.msg.Joy()         # currents 
forces_joy   = sensor_msgs.msg.Joy()         # strain-gauge forces (raw)

previous_positions_stamp = None
previous_torques_stamp = None
previous_torques = np.zeros( n_joints )
previous_positions = np.zeros( n_joints )



# IMU names and messages
#
# imu_names = ['palm_imu', 'LF_imu', 'RF_imu' ] # palm IMU on Arduino nano 33 IOT, two IMUs on middle finger and thumb
imu_names = ['palm_imu' ] # only the main 'palm' IMU
imu_accel_scale = []
imu_gyro_scale = []
imu_messages = []
imu_raw_joy_messages = []
imu_calibration = []

for i in range( len(imu_names) ):
    imu_raw_joy_messages.append( sensor_msgs.msg.Joy() )
    imu_messages.append( sensor_msgs.msg.Imu() )
    imu_calibration.append( {} )
    imu_accel_scale.append( 1.0 )
    imu_gyro_scale.append( 1.0 )
# print( '>>>>>>> len(imu) ' + str( len( imu_calibration )))


# default servo calibration (including cw/ccw rotation direction)
# 
LFJ1 = {}
LFJ1[ -90 ] =  2120-1024
LFJ1[   0 ] =  2120
LFJ1[ +90 ] =  2120+1024

LFJ2 = {}
LFJ2[ -90 ] =  2048-1024
LFJ2[   0 ] =  2048
LFJ2[ +90 ] =  2048+1024

RFJ1 = {}
RFJ1[ -90 ] =  2008+1024
RFJ1[   0 ] =  2008
RFJ1[ +90 ] =  2008-1024

RFJ2 = {}
RFJ2[ -90 ] =  2048+1024
RFJ2[   0 ] =  2048
RFJ2[ +90 ] =  2008-1024

joint_offsets = [ 2120, 2048, 2008, 2048 ]
joint_scale   = [ 1024.0/90.0, 1024.0/90.0, 1024.0/90.0, 1024.0/90.0 ] # degrees
vel_scale     = [ 1.0/4096, 1.0/4096, 1.0/4096, 1.0/4096 ]  # SMS servo 4096 steps/2pi SCS 1024 steps/2pi
servo_signs   = [ +1, +1, -1, -1 ]

mutex = threading.Lock()
joint_state_goal= sensor_msgs.msg.JointState()
have_new_motion_goal = False


def getp( j, counts ):  # joint_index and raw position counts 
     value = DEG2RAD*( (counts - joint_offsets[j]) * servo_signs[j] / joint_scale[j])
     return value


def getv( j, counts ):
     value = DEG2RAD * counts * vel_scale[j]
     return value


def radians_to_counts( j, radians ): # map ROS angle (rad) to SCS servo #j counts
     value = int( joint_offsets[j] + (radians * RAD2DEG)*servo_signs[j]*joint_scale[j] )
     print( "radians_to_counts: joint " + str(j) + " radians " + str(radians)
            + " -> " + str( value ))

     return value


def handle_text_command( request ):
    global text_command
    global have_text_command
    print( "####################" )
    print( "handle_text_command: Received '" + str( request.command ))

    if (have_text_command != 0):
       print( "handle_text_command: Busy; command ignored!" )
       return False

    else:
       text_command = request.command
       have_text_command = 1

    res = diana_gripper.srv.TextCommandResponse()
    res.status = "ok."
    print( "####################" )
    return res


def simple_goal_callback( msg ):
    global have_new_motion_goal
    global joint_state_goal
    # rospy.loginfo( rospy.get_called_id() + " Got new simple joint goal " + str(msg) )
    print( "Got new simple goal " + str(msg) )

    try: 
        mutex.acquire()
        if (have_new_motion_goal):
            rospy.logwarn( "Got new motion goal while previous was not processed, overwriting..." );

        # for the "simple" joint goal, we take the first joint position
        # and apply it to all four joints (with signs adjusted as necessary).
        # If position[1] is also given, that is used as the "delta" angle
        # for the distal phalanges (fingertips tilt inwards or outwards).
        # The rest of the message, including velocities and efforts, is ignored.
        #
        # Maybe it would be nice to take an "opening" width in meters and
        # convert to joint-angle via IK?
        # 
        angle =  msg.position[0]

        delta = 0.0
        if len(msg.position) > 1:
            delta = msg.position[1]

        tilt = 0.0
        if len(msg.position) > 2:
            tilt = msg.position[2]

        joint_state_goal.position = [angle+delta, -angle+tilt,  angle+delta, -angle+tilt]

        joint_state_goal.header.stamp = msg.header.stamp
        joint_state_goal.velocity = []
        joint_state_goal.effort = []
        have_new_motion_goal = True

    finally:
        mutex.release()
    return



def joint_goal_callback( msg ):
    global have_new_motion_goal
    global joint_state_goal
    # rospy.loginfo( rospy.get_called_id() + " Got new joint goal " + str(msg) )
    print( "Got new joint goal " + str(msg) )

    try: 
        mutex.acquire()
        if (have_new_motion_goal):
            rospy.logwarn( "Got new motion goal while previous was not processed, overwriting..." );

        # some ROS nodes use urdf order, some alphabetic, and then some.
        # Fact is, we need to (re-) order the joints here from whatever 
        # the sending node uses to our internal numbering scheme
        joint_state_goal.position = [0,0,0,0]
        for i in range( len(msg.name) ):
            joint_name = msg.name[i]
            ji = joint_index[ joint_name ]
            joint_state_goal.position[ji] = msg.position[i]
            print( ">>>>>>>>>> new joint goal[" + str(ji) + " ] -> " + str(msg.position[i]) )

            # skip velocity and effort for now

        joint_state_goal.header = msg.header
        joint_state_goal.velocity = []
        joint_state_goal.effort = []
        have_new_motion_goal = True

    finally:
        mutex.release()
    return




def diana_gripper():
    global verbose, js
    global have_text_command, text_command
    global have_new_motion_goal, joint_state_goal
    global previous_positions, previous_positions_stamp
    global previous_torques, previous_torques_stamp

    global n_sensors, forces_bias, forces_gains
    global set_zero_bias_in_progress, set_zero_bias_start_time, set_zero_bias_interval, set_zero_bias_channels
    global set_gains_in_progress, set_gains_tokens
    print( "diana_gripper init...\n" )

    # UDP Socket
    # ARDUINO_IP = "192.168.104.97";   # unused, hardcoded in INFFUL network setup
    # UDP_IP = "134.100.13.101"        # our own IP address
    # UDP_PORT = 11333;                # the port we use, must match Arduino settings
    # 
    # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    # sock.bind((UDP_IP, UDP_PORT))
    # print( "got the UDP socket..." )

    # ROS node stuff
    #
    rospy.init_node( 'diana_gripper', anonymous=False )

    if rospy.has_param( '~verbose' ):
        verbose = rospy.get_param( '~verbose' )

    rate_hz = 500.0; 
    if rospy.has_param( '~rate' ):
        rate_hz = rospy.get_param( '~rate' )
    rate = rospy.Rate( rate_hz )
    print( '... loop rate is ' + str(rate_hz) + ' hz.' )

    # Serial-port stuff 
    # 
    # port_name = '/dev/ttyUSB0'  # Arduino nano, ESP32 NodeMCU, ...
    port_name = '/dev/ttyACM1'  # Teensy, Nano 33 IOT, Feather Wifi ...  
  
    if rospy.has_param( '~port_name' ):
        port_name = rospy.get_param( '~port_name' )
        print( '... serial port name is ' + port_name )

    baudrate = 115200
    if rospy.has_param( '~baudrate' ):
        baudrate = rospy.get_param( '~baudrate' )

    ser = serial.Serial( port_name, baudrate=baudrate )
    print( "... got the serial port ('" + port_name + ") baudrate '" + str(baudrate) )

    #
    # IMU calibration: ranges/scales and offsets/biases
    #
    for i in range( len( imu_names )):
        if (verbose > 1): print( '... checking calibration parameters for IMU ' + imu_names[i] )
        nnames = ['gyro_scale', 'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z', 'accel_scale', 'accel_bias_x', 'accel_bias_y', 'accel_bias_z']
        nvals  = [ 250.0*DEG2RAD/32768.0, 0.0, 0.0, 0.0, (2.0*9.81/32768.0), 1.0, 1.0, 1.0 ]

        for n in range( len( nnames )):
            pname = 'ugly_hand/' + imu_names[i] + '/' + nnames[n]
            if (verbose > 2): print( '... checking param ' + pname )
            if rospy.has_param( pname ):
                value = rospy.get_param( pname )
                imu_calibration[ i ][ nnames[n] ] = value
                print( '... param ' + pname + ' = ' + str(value) )
                # print( imu_calibration[ i ][ nnames[n] ] )
            else:
                imu_calibration[ i ][ nnames[n] ] = 0.0
                value = nvals[ n ]
                if (verbose > 2): print( '... param ' + pname + ' = ' + str(value) )


    # ROS publishers and subscribers
    # 
    command_server = rospy.Service( '~command', diana_gripper.srv.TextCommand,
                                    handle_text_command )
    print( "... created the text-command server ..." )

    iteration = 0
    running = 1
    joint_state_publisher = rospy.Publisher( '~joint_states', sensor_msgs.msg.JointState, queue_size=2 ) 
    joint_error_publisher = rospy.Publisher( '~joint_errors', sensor_msgs.msg.JointState, queue_size=2 ) 

    torques_publisher     = rospy.Publisher( '~torques',      sensor_msgs.msg.Joy, queue_size=2 )
    temperature_publisher = rospy.Publisher( '~temperatures', sensor_msgs.msg.Joy, queue_size=2 )
    voltages_publisher    = rospy.Publisher( '~voltages',     sensor_msgs.msg.Joy, queue_size=2 )
    currents_publisher    = rospy.Publisher( '~currents',     sensor_msgs.msg.Joy, queue_size=2 )
    forces_raw_publisher  = rospy.Publisher( '~forces_raw',   sensor_msgs.msg.Joy, queue_size=2 )
    forces_publisher      = rospy.Publisher( '~forces',       sensor_msgs.msg.Joy, queue_size=2 )

    # IMU publishers (raw data as Joy message, imu data as Imu message, no quaternion fusion yet)
    # 
    imu_publishers = []
    imu_raw_joy_publishers = []
    for i in range( len( imu_names )):
        imu_raw_joy_publishers.append(
            rospy.Publisher( '~' + imu_names[i] + '/imu_joy', sensor_msgs.msg.Joy, queue_size=2 ))
        imu_publishers.append(
            rospy.Publisher( '~' + imu_names[i] + '/imu_raw', sensor_msgs.msg.Imu, queue_size=2 ))

    # goal subscribers
    # 
    joint_goal_subscriber = rospy.Subscriber( '~joint_goals', sensor_msgs.msg.JointState, joint_goal_callback )
    simple_goal_subscriber = rospy.Subscriber( '~simple_goal', sensor_msgs.msg.JointState, simple_goal_callback )
    print( "got the ROS node and publishers..." )

    # start ROS main loop
    # 
    while running and not rospy.is_shutdown():
        iteration = iteration + 1
        if (verbose > 3): 
            print( "diana_gripper loop iteration " + str(iteration) + "..." )

        try:
            if (have_text_command == 1):
                n_written = ser.write( str.encode( text_command + '\n' ) )
                ser.flush()
                have_text_command = 0
                if (verbose > 2):
                     print( "### wrote command '" + str(text_command) + "'..." )

            elif (have_new_motion_goal):
                if verbose > 4: print( "### have_new_motion_goal..." )
                try:
                    mutex.acquire()
                    # extract new motion goal and send via serial port TODO IMPLEMENT
                    # joint_state_goal
                    j0 = radians_to_counts( 0, joint_state_goal.position[0] )
                    j1 = radians_to_counts( 1, joint_state_goal.position[1] )
                    j2 = radians_to_counts( 2, joint_state_goal.position[2] )
                    j3 = radians_to_counts( 3, joint_state_goal.position[3] )

                    # j0 = int( joint_state_goal.position[0] )
                    # j1 = int( joint_state_goal.position[1] )
                    # j2 = int( joint_state_goal.position[2] )
                    # j3 = int( joint_state_goal.position[3] )
                    ser.write( str.encode( 'G ' + str(j0) + ' ' + str(j1) + ' ' + str(j2) + ' ' + str(j3) + ' \n' ) )
                    ser.flush()
                    have_new_motion_goal = False

                finally:
                    mutex.release()

            data = ser.readline().decode( "utf-8" ) ## stupid python3 
            if (verbose > 2) or ((iteration / 1000) == 999):
                print( "received message: ", data);

            if (data[0] == 'D'): # old style joint_state packet

                (str_D, seq_number, stamp, str_J, p1, p2, p3, p4, str_V, v1, v2, v3, v4, str_E, e1, e2, e3, e4) = \
                [t(s) for t,s in zip(( str, int, int, 
                                      str, int, int, int, int,
                                      str, int ,int ,int, int,
                                      str, int, int, int, int ), data.split()) ]

                js.header.frame_id = 'diana_gripper'
                js.header.stamp    = rospy.Time.now()
                js.name            = joint_names
                js.position        = [getp(0,p1), getp(1,p2), getp(2,p3), getp(3,p4)]
                js.velocity        = [getv(0,v1), getv(1,v2), getv(2,v3), getv(3,v4)]
                js.effort          = [e1, e2, e3, e4]

                joint_state_publisher.publish( js )

            elif (data[0] == 'I'): # servo currents
                (str_I, seq_number, stamp, str_amps, u1, u2, u3, u4 ) = \
                [t(s) for t,s in zip(( str, int, int, str, int, int, int, int), data.split()) ]

                curr_joy.header.frame_id = 'diana_gripper/servo_currents'
                curr_joy.header.stamp    = rospy.Time.now()
                curr_joy.axes            = []
                curr_joy.axes.append( u1 )
                curr_joy.axes.append( u2 )
                curr_joy.axes.append( u3 )
                curr_joy.axes.append( u4 )
                currents_publisher.publish( curr_joy )

            elif (data[0] == 'E'): # servo position errors: 'b'E 102 166 pos 0 0 0 0\n''
                (str_E, seq_number, stamp, str_pos, err1, err2, err3, err4 ) = \
                  [t(s) for t,s in zip(( str, int, int, str, int, int, int, int), data.split()) ]

                # joint_position_errors packet: abuse JointState so that we has pos,vel,pos-error in one message
                # FIXME: this currently uses/publishes RAW values (scservo counts), but SHOULD publish
                # SI units (rad, rad/s, delta rad)
                #
                js_errors.header.frame_id = 'diana_gripper/joint_pos_joint_goals_pos_errors'
                js_errors.header.stamp    = rospy.Time.now()
                js_errors.name            = joint_names
                js_errors.position        = [err1, err2, err3, err4]
                for i in range( n_joints):
                    try:
                        js_errors.velocity[i] = (joint_state_goal.velocity[i] - js.velocity[i])
                        js_errors.effort[i]   = (joint_state_goal.effort[i] - js.effort[i])
                    except:
                        js_errors.velocity[i] = math.nan
                        js_errors.effort[i] = math.nan

                joint_error_publisher.publish( js_errors )

            elif (data[0] == 'F'): # loadcell forces
                (str_F, seq_number, stamp, str_forces, f1, f2, f3, f4 ) = \
                [t(s) for t,s in zip(( str, int, int, str, int, int, int, int), data.split()) ]

                forces_joy.header.frame_id = 'diana_gripper/loadcell_forces_raw'
                forces_joy.header.stamp    = rospy.Time.now()
                forces_joy.axes            = []
                forces_joy.axes.append( f1 )
                forces_joy.axes.append( f2 )
                forces_joy.axes.append( f3 )
                forces_joy.axes.append( f4 )
                forces_raw_publisher.publish( forces_joy )

            elif (data[0] == 'G'): # IMU: 'b'G 88 173 0 accel -5 -37 8235 gyro 13 -47 -59 \n''
                (str_I, seq_number, stamp, imu_index, str_accel, ax, ay, az, str_gyro, gx, gy, gz ) = \
                  [t(s) for t,s in zip(( str, int, int, int, str, int, int, int, str, int, int, int), data.split()) ]

                joy = imu_raw_joy_messages[ imu_index ]
                joy.header.frame_id = hand_name + '/' + imu_names[ imu_index ] + '/frame'
                joy.header.stamp    = rospy.Time.now()
                joy.axes            = []
                joy.axes.append( ax )
                joy.axes.append( ay )
                joy.axes.append( az )
                joy.axes.append( gx )
                joy.axes.append( gy )
                joy.axes.append( gz )
                imu_raw_joy_publishers[imu_index].publish( joy )

                # FIXME: replace hardcoded values for MPU6050 with params
                accel_scale = 2.0 * 9.81 / 32768.0   #  2g max at 16-bit signed
                gyro_scale  = 250.0 * DEG2RAD / 32768  # 250 deg/s max at 16 bit signed

                imu_msg = imu_messages[ imu_index ]

                imu_msg.header.frame_id = hand_name + '/' + imu_names[ imu_index ] + '/frame'
                imu_msg.header.stamp    = joy.header.stamp
                imu_msg.linear_acceleration.x = ax * accel_scale
                imu_msg.linear_acceleration.y = ay * accel_scale
                imu_msg.linear_acceleration.z = az * accel_scale
                imu_msg.angular_velocity.x = gx * gyro_scale - imu_calibration[ imu_index ]['gyro_bias_x']
                imu_msg.angular_velocity.y = gy * gyro_scale - imu_calibration[ imu_index ]['gyro_bias_y']
                imu_msg.angular_velocity.z = gz * gyro_scale - imu_calibration[ imu_index ]['gyro_bias_z']
                imu_publishers[0].publish( imu_msg )

            elif (data[0] == 'I'): # servo currents
                (str_I, seq_number, stamp, str_amps, u1, u2, u3, u4 ) = \
                [t(s) for t,s in zip(( str, int, int, str, int, int, int, int), data.split()) ]

                curr_joy.header.frame_id = 'diana_gripper/servo_currents'
                curr_joy.header.stamp    = rospy.Time.now()
                curr_joy.axes            = []
                curr_joy.axes.append( u1 )
                curr_joy.axes.append( u2 )
                curr_joy.axes.append( u3 )
                curr_joy.axes.append( u4 )
                currents_publisher.publish( curr_joy )

            elif (data[0] == 'M'): # servo torques (_m_oments): 'b'M 93 168 trq -1 -1 -1 -1\n''
                (str_M, seq_number, stamp, str_trq, u1, u2, u3, u4 ) = \
                [t(s) for t,s in zip(( str, int, int, str, int, int, int, int), data.split()) ]

                torques_joy.header.frame_id = 'diana_gripper/servo_torques'
                torques_joy.header.stamp    = rospy.Time.now()
                torques_joy.axes            = []
                torques_joy.axes.append( u1 )
                torques_joy.axes.append( u2 )
                torques_joy.axes.append( u3 )
                torques_joy.axes.append( u4 )
                torques_publisher.publish( torques_joy )


            elif (data[0] == 'P'): # servo positions: 'b'P 92 167 pos -1 -1 -1 -1\n''
                 # servo/joint positions packet (raw counts),
                 # velocities calculated from numerical difference,
                 # efforts used from last torque packet

                (str_P, seq_number, stamp, str_pos, p1, p2, p3, p4 ) = \
                  [t(s) for t,s in zip(( str, int, int, str, int, int, int, int ), data.split()) ]

                js.header.frame_id = 'diana_gripper' # 'diana_gripper'
                js.header.stamp    = rospy.Time.now()
                js.name            = joint_names
                current_positions  = [getp(0,p1), getp(1,p2), getp(2,p3), getp(3,p4) ]

                js.position        = current_positions
                js.effort          = previous_torques

                # js.velocity        = [getv(0,v1), getv(1,v2), getv(2,v3), getv(3,v4)]
                # 
                if previous_positions_stamp is not None:
                    delta_t            = (js.header.stamp - previous_positions_stamp).to_sec()
                    delta_millis       = (stamp - js_millis) / 1000.0
                    if (delta_millis < 0): delta_millis = delta_millis + 0.256  # millis() & 0xff
                    if verbose > 4:
                        print( "... delta_t pos: ROS " + str( delta_t ) + "  Arduino " + str( delta_millis ) + " raw diff " + str((stamp - js_millis)/1000.0) );
                    if abs(delta_t - delta_millis) > 0.02:
                        rospy.logerr( "... delta_t pos: ROS " + str( delta_t ) + "  Arduino " + str( delta_millis ));

                    # js.velocity        = (np.array(current_positions) - np.array(previous_positions)) / delta_tdd
                    js.velocity        = (np.array(current_positions) - np.array(previous_positions)) / delta_millis
                else:
                    js.velocity        = np.zeros( n_joints )

                previous_positions = current_positions
                previous_positions_stamp = js.header.stamp
                js_millis = stamp
                joint_state_publisher.publish( js )

            elif (data[0] == 'T'): # motor temperatures
                (str_T, seq_number, stamp, str_t, t1, t2, t3, t4 ) = \
                [t(s) for t,s in zip(( str, int, int, str, int, int, int, int), data.split()) ]

                temp_joy.header.frame_id = 'diana_gripper/servo_temperatures'
                temp_joy.header.stamp    = rospy.Time.now()
                temp_joy.axes            = []
                temp_joy.axes.append( t1 )
                temp_joy.axes.append( t2 )
                temp_joy.axes.append( t3 )
                temp_joy.axes.append( t4 )
                temperature_publisher.publish( temp_joy )

            elif (data[0] == 'U'): # servo voltages
                (str_U, seq_number, stamp, str_volts, u1, u2, u3, u4 ) = \
                [t(s) for t,s in zip(( str, int, int, str, int, int, int, int), data.split()) ]

                volt_joy.header.frame_id = 'diana_gripper/servo_voltages'
                volt_joy.header.stamp    = rospy.Time.now()
                volt_joy.axes            = []
                volt_joy.axes.append( u1 )
                volt_joy.axes.append( u2 )
                volt_joy.axes.append( u3 )
                volt_joy.axes.append( u4 )
                voltages_publisher.publish( volt_joy )

            else: 
                print( "Unknown data/packet ignored: '" + str(data) + "'" )

        except ValueError:
             print( "Parsing data failed: '" + str(data) + "'" )
             continue

        except KeyboardInterrupt:
             running = False
             pass

        rate.sleep()

    ser.close()
    print( "diana_gripper node stopped." )
    


if __name__ == '__main__':
    try:
        diana_gripper()
    except KeyboardInterrupt:
        print( "Received control-c, stopping..." )
        exit( 0 )
    except rospy.ROSInterruptException:
        pass

