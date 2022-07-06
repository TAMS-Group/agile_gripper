#! /usr/bin/env python 
# diana_gripper_fjtas.py - basic FollowJointTrajectoryActionServer
# using SCServo position control (and optionally, velocity mode).
#
# Example Moveit-generated goal:
#  header: 
#    seq: 4
#    stamp: 
#      secs: 1623232077
#      nsecs: 334722081
#    frame_id: ''
#  goal_id: 
#    stamp: 
#      secs: 1623232077
#      nsecs: 334723049
#    id: "/move_group-5-1623232077.334723049"
#  goal: 
#    trajectory: 
#      header: 
#        seq: 0
#        stamp: 
#          secs: 0
#          nsecs:         0
#        frame_id: "/world"
#      joint_names: [diana_gripper/LFJ1, diana_gripper/LFJ2, diana_gripper/RFJ1, diana_gripper/RFJ2]
#      points: 
#      - 
#       positions: [-0.7133010663668231, -0.0015339807878856412, 0.7853981633974483, 0.0030679615757712823]
#       velocities: [0.0, 0.0, 0.0, 0.0]
#       accelerations: [0.0, -4.573432270722696e-05, 0.0, 0.0]
#       effort: []
#       time_from_start: 
#         secs: 0
#         nsecs:         0

# 2022.07.04 - update (from qbsc_gripper_fjtas)
# 2021.06.09 - implement and first test
# 2021.06.08 - created
#
# (c) 2021, 2022 fnh, hendrich@informatik.uni-hamburg.de
#

import collections
import copy
import math
import numpy as np
import threading   # want mutex

import rospy
import actionlib
import std_msgs.msg
import sensor_msgs.msg
import control_msgs.msg 

from IPython import embed

print( 'diana_gripper_fjtas: global init started...' )

mutex = threading.Lock()

hand_prefix = 'diana_gripper'
short_joint_names = ['LFJ1', 'LFJ2', 'RFJ1', 'RFJ2' ]  # left finger distal + proximal, right finger distal+proximal
n_joints = 4
joint_names = []
joint_index_map = {}
for i in range( len( short_joint_names )): 
    full_joint_name = hand_prefix + '/' + short_joint_names[i]
    joint_names.append( full_joint_name )
    joint_index_map[ short_joint_names[i] ] = i
    joint_index_map[ full_joint_name ] = i
four_zeros = [0.0, 0.0, 0.0, 0.0]

print( joint_index_map )
print( 'diana_gripper_fjtas: global init ok.' )


class DianaGripperFollowJointTrajectoryAction( object ):

    # embed()

    def __init__(self, name):
        print( '... diana_gripper trajectory action controller (fjtas action).init...' )
    
        print( "###########################################################" )
        print( "DianaGripperFollowJointTrajectoryFCUCKAÖFKDLJALSKFJSALKFJKSDLFJ" )
        print( n_joints )
    
        self.joint_state          = sensor_msgs.msg.JointState()
        self.joint_state.name     = joint_names
        self.joint_state.position = np.zeros( n_joints )
        self.joint_state.velocity = np.zeros( n_joints )
        print( self.joint_state.position )
    
        self.joint_goal          = sensor_msgs.msg.JointState()
        self.joint_goal.name     = joint_names
        self.joint_goal.position = np.zeros( n_joints )
        self.joint_goal.velocity = np.zeros( n_joints )
        print( "###########################################################" )
    
        self.n_joint_state_messages = 0

        # create messages that are used to publish feedback and result
        self.feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
        self.result   = control_msgs.msg.FollowJointTrajectoryResult()
        self.verbose  = 2

        # 
        self.joint_states_subscriber = rospy.Subscriber( '/diana_gripper/joint_states',
                                                         sensor_msgs.msg.JointState, 
                                                         self.joint_states_callback )
        while self.n_joint_state_messages < 5:
            print( '... waiting for /diana_gripper/joint_states to appear... (received ' 
                   + str( self.n_joint_state_messages) + ' messages)' )
            rospy.sleep( rospy.Duration( 5.0 ) )
        print( '... subscribed to /diana_gripper/joint_states...' )
    
        # create joint_goals publisher and wait for it to become ready...
        self.joint_goal_publisher = rospy.Publisher( '/diana_gripper/joint_goals', sensor_msgs.msg.JointState, queue_size=1 )
        while self.joint_goal_publisher.get_num_connections() < 1:
            print( '... waiting for /diana_gripper/joint_goals publisher to connect...' )
            rospy.sleep( rospy.Duration( 5.0 ))
        print( '... publisher connected to /diana_gripper/joint_goals...' )

        # finally, initalize the action server...
        self.action_name = name
        self.action_server = actionlib.SimpleActionServer(
                      self.action_name, 
                      control_msgs.msg.FollowJointTrajectoryAction, 
                      execute_cb = self.execute_callback, auto_start = False )
        self.action_server.start()
        print( '... action server started, ready...' )


    def joint_states_callback( self, msg ):
        if self.verbose >= 3:
            print( "DianaGripperFJTAS.joint_states_callback: " + str( self.n_joint_state_messages ) + " " + str(msg) )
        self.n_joint_state_messages = self.n_joint_state_messages + 1

        # print( "GABI" + str( self.joint_state ))

        try:
            mutex.acquire()
            # some ROS nodes use urdf order, some alphabetic, and then some.
            # Fact is, we need to (re-) order the joints here from whatever 
            # the sending node uses to our internal numbering scheme
            for i in range( len(msg.name) ):
                joint_name = msg.name[i]
                ji = joint_index_map[ joint_name ]
                self.joint_state.position[ji] = msg.position[i]

                if self.verbose >= 4:
                    print( "... joint_states position[" + str(ji) + " ] -> " + str(msg.position[i]) )

                if len(msg.velocity) == len(msg.position):
                    self.joint_state.velocity[ji] = msg.velocity[i]

                # skip effort for now
            self.joint_state.header = msg.header
            # FIXME: there must b other variables that need to be kept?

        finally:
            mutex.release()
        return

    def move_to_trajectory_point( self, goal, pp ):
        global joint_index_map

        print( '... move_to_trajectory_point ' + str( pp ) + ' ...' )
        self.joint_goal.header.stamp = rospy.Time.now()
        self.joint_goal.header.frame_id = 'test_test_test'  # world?
        try:
            mutex.acquire()
            self.joint_goal.position = self.joint_state.position
            self.joint_goal.velocity = self.joint_state.velocity
            # skip effort 
        finally:
            mutex.release()
 
        print( type( goal ))
        print( self.joint_goal )
 
        # Warning: moveit sends partial trajectories, with only those joints included in a planning group 
        ji = 0
        for i in range( len( goal.trajectory.joint_names )):
            ji = joint_index_map[ goal.trajectory.joint_names[i] ]
            # print( ji )
            # print( pp )
            # print( i )

            self.joint_goal.position[ji] = goal.trajectory.points[pp].positions[i]
            self.joint_goal.velocity[ji] = goal.trajectory.points[pp].velocities[i]
            # skip accelerations 
            # skip efforts
        self.joint_goal_publisher.publish( self.joint_goal )
        return


    def execute_callback( self, goal ):
        if self.verbose >= 0:
            print( "DianaGripperFJTAS.execute_callback... n_points = " + str( len( goal.trajectory.points )) + " ... " )
            print( type( goal ))
        if self.verbose >= 3:
            print( goal )

        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self.feedback.header.stamp = rospy.Time.now()
        self.feedback.header.frame_id = "HUGOO!!" 
        self.feedback.joint_names = joint_names 

        self.feedback.desired.positions = copy.deepcopy( four_zeros )
        self.feedback.desired.velocities = copy.deepcopy( four_zeros )
        self.feedback.desired.accelerations = copy.deep_copy( four_zeros )
        self.feedback.desired.time_from_start = rospy.Time.now()

        self.feedback.actual.positions = copy.deepcopy( four_zeros )
        self.feedback.actual.velocities = copy.deepcopy( four_zeros )
        self.feedback.actual.accelerations = copy.deepcopy( four_zeros )
        self.feedback.actual.time_from_start = rospy.Time.now()

        self.feedback.error.positions = copy.deepcopy( four_zeros )
        self.feedback.error.velocities = copy.deepcopy( four_zeros )
        self.feedback.error.accelerations = copy.deepcopy( four_zeros )
        self.feedback.error.time_from_start = rospy.Time.now()

        rospy.loginfo('Executing, creating trajectory point %i with seeds %i, %i' % (self.verbose, 3, 14 ))

        t_now   = rospy.Time.now()
        t_start = rospy.Time.now() + goal.trajectory.points[0].time_from_start
        t_end   = rospy.Time.now() + goal.trajectory.points[-1].time_from_start
        print( '... Trajectory t_start ' + str( t_start.to_sec() ) + ' t_end ' + str( t_end.to_sec() )
               + ' duration ' + str( (t_end - t_start).to_sec() ))

        pp = 0
        ppmax = len( goal.trajectory.points )
        t_pp = t_start + goal.trajectory.points[ pp ].time_from_start
        while t_now < t_end:
            while (pp < ppmax) and (t_pp <= t_now):
                pp = pp + 1
                t_pp = t_start + goal.trajectory.points[ pp ].time_from_start

            # publish info to the console for the user
            print( '... trajectory t= ' + str( t_now.to_sec() ) + '  point index ' + str( pp ))

            # execute the trajectory point
            self.move_to_trajectory_point( goal, pp )

            rospy.sleep( rospy.Duration( 0.05 )) 
            t_now = rospy.Time.now()

            # send joint_goals to servos:

            # periodically send feedback messages:
        print( '... trajectory FINISHED.' )
        

        if success: 
            # does NOT work...
            # self.result.header = goal.trajectory.header
            # self.result.header.stamp = rospy.Time.now()
            # self.result.status = self.result.LOST

            self.result.error_code = self.result.GOAL_TOLERANCE_VIOLATED
            self.result.error_string = 'Hiho!!!'
            # self.result.goal_id = goal.goal_id HOW TO ACCESS the goal_id ????

            rospy.loginfo('%s: Succeeded' % self.action_name)
            self.action_server.set_succeeded(self.result)




        
if __name__ == '__main__':
    rospy.init_node('diana_gripper_fjtas', anonymous=False, disable_signals=True ) 
    fjtas = DianaGripperFollowJointTrajectoryAction( "/diana_gripper_controller/follow_joint_trajectory_action" )

    #  rospy.spin() # IMPOSSIBLE to control-c this crap under python3, simply continues running 
    while not rospy.is_shutdown():
        rospy.sleep( rospy.Duration( 0.1 ))
    exit( 0 )
