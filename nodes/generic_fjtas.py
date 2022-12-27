#! /usr/bin/env python 
# generic_fjtas.py - a basic FollowJointTrajectoryActionServer that 
# republishes joint-trajectory points on a JointState topic in real time.
#
# There are three typical uses cases for this node:
# (a) to accept JointTrajectory messages for a robot that only offers
#     JointState (position or velocity) control inputs, 
# (b) to accept JointTrajectory messages for a robot that only offers
#     a set of single (per-joint) position controllers,
# (c) to visualize JointTrajectory messages (e.g. from Moveit) during
#     trajectory execution (in real-time) with Plotjuggler or similar.
# Output is either a combined ~joint_goals (JointState) message or a
# list of per-joint "/namespace/joint-name/command" output publishers.
#
# Typical usage (remap topics as needed):
# 1. configure the joint-names for this node
# 2. subscribe to ~joint_states and wait for ~joint_goals
# 3. republish incoming joint-trajectory goals on ~joint_goals
#
# /joint_states:  incoming joint_states from the robot, this might
#                 include joint-names ignored by this node
# /joint_goals:   outgoing joint_states corresponding to the active
#                 trajectory point (if any)
# /namespace/joint-name/command: outgoing command topic, one per
#                 activated joint
# /node_name/fjtas/goal ...    : topics created by the ActionServer
# 
# using SCServo position control (and optionally, velocity mode).
#     FollowJointTrajectoryAction

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

# 2022.12.27 - create "generic" version, takes joint-names from ROS parameter
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

# from IPython import embed


print( 'generic_fjtas: global init started...' )
verbose = 17
mutex = threading.Lock()




# joint_names = []
# print( joint_index_map )
# print( 'diana_gripper_fjtas: global init ok.' )
# all_zeros = [0.0, 0.0, 0.0, 0.0]


class GenericFollowJointTrajectoryAction( object ):

    # embed()

    def __init__(self, name, joint_names ):
        print( '... generic follow joint trajectory action controller (fjtas action).init...' )
    
        # print( "###########################################################" )
        if verbose > 5: print( "... joint_names: " + str( joint_names ))

        self.t_sleep              = 0.01    # useful default
        if rospy.has_param( "rate" ):
            rate = rospy.get_param( "rate" )
            self.t_sleep = 1.0 / rate
            print( "... rate: " + str( rate ) + " -> t_sleep: " + str( self.t_sleep ))

        self.n_joints             = len( joint_names )
        self.joint_names          = joint_names

        self.joint_state          = sensor_msgs.msg.JointState()
        self.joint_state.name     = joint_names
        self.joint_state.position = np.zeros( self.n_joints )
        self.joint_state.velocity = np.zeros( self.n_joints )
    
        self.joint_goal          = sensor_msgs.msg.JointState()
        self.joint_goal.name     = joint_names
        self.joint_goal.position = np.zeros( self.n_joints )
        self.joint_goal.velocity = np.zeros( self.n_joints )

        self.make_joint_index_map()


        # print( "###########################################################" )
    
        self.n_joint_state_messages = 0

        # create messages that are used to publish feedback and result
        self.feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
        self.result   = control_msgs.msg.FollowJointTrajectoryResult()
        self.verbose  = 2

        self.joint_states_subscriber = rospy.Subscriber( '~joint_states',
                                                         sensor_msgs.msg.JointState, 
                                                         self.joint_states_callback )
        while self.n_joint_state_messages < 5:
            rospy.logwarn_throttle( 5.0, '... waiting for ~joint_states to appear... (received ' 
                   + str( self.n_joint_state_messages) + ' messages)' )
            rospy.sleep( rospy.Duration( 0.2 ) )
        print( '... subscribed to ~joint_states now...' )
    
        # create joint_goals publisher and wait for it to become ready...
        self.joint_goal_publisher = rospy.Publisher( '~joint_goals', sensor_msgs.msg.JointState, queue_size=1 )
        while self.joint_goal_publisher.get_num_connections() < 1:
            rospy.logwarn_throttle( 5.0, '... waiting for ~joint_goals subscriber to connect...' )
            rospy.sleep( rospy.Duration( 0.2 ))
        print( '... publisher connected to ~joint_goals now...' )

        # finally, initalize the action server...
        self.action_name = name
        self.action_server = actionlib.SimpleActionServer(
                      self.action_name, 
                      control_msgs.msg.FollowJointTrajectoryAction, 
                      execute_cb = self.execute_callback, auto_start = False )
        self.action_server.start()
        print( '... action server started, ready...' )
        return


    def make_joint_index_map( self ):
        self.joint_index_map = {}
        for i in range( len( self.joint_names )):
            self.joint_index_map[ self.joint_names[i] ] = i
        return




    def joint_states_callback( self, msg ):
        if self.verbose >= 3:
            print( "GenericFJTAS.joint_states_callback: " + str( self.n_joint_state_messages ) + " " + str(msg) )
        self.n_joint_state_messages = self.n_joint_state_messages + 1

        # print( "GABI" + str( self.joint_state ))

        try:
            mutex.acquire()
            # some ROS nodes use urdf order, some alphabetic, and then some.
            # Fact is, we need to (re-) order the joints here from whatever 
            # the sending node uses to our internal numbering scheme
            for i in range( len(msg.name) ):
                joint_name = msg.name[i]
                if joint_name not in self.joint_index_map:
                    continue
                ji = self.joint_index_map[ joint_name ]
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

    # linear interpolation between two goal trajectory points at indices [i0] and [i1].
    # Returns a both the position and the velocities as two vectors in a tuple.
    # No reordering is done on the trajectory point elements.
    # 
    def interpolate_point( self, TP0, TP1,  fraction ):
        P0 = np.array( TP0.positions )
        P1 = np.array( TP1.positions )
        V0 = np.array( TP0.velocities )
        V1 = np.array( TP1.velocities )

        PP = P0 + fraction*(P1-P0)
        VV = V0 + fraction*(V1-V0)

        return (PP, VV)


    def move_to_interpolated_point( self, goal, PP, VV ):
        # global joint_index_map

        if self.verbose > 4:
            print( '... move_to_interpolated_point...' + str(PP) )
    
        # initialize new goal with current robot joint_state
        # 
        self.joint_goal.header.stamp = rospy.Time.now()
        self.joint_goal.header.frame_id = 'diana_gripper_fjtas'  # world?
        try:
            mutex.acquire()
            self.joint_goal.position = self.joint_state.position
            self.joint_goal.velocity = self.joint_state.velocity
            # skip effort 
        finally:
            mutex.release()
 
        # Need to reorder trajectories generated by MoveIt, as those might have
        # any subset of actual robot joints in any order, corresponding to the
        # active planning group...
        # 
        ji = 0
        for i in range( len( goal.trajectory.joint_names )):
            ji = self.joint_index_map[ goal.trajectory.joint_names[i] ]

            self.joint_goal.position[ji] = PP[i]
            self.joint_goal.velocity[ji] = VV[i]
            # skip accelerations 
            # skip efforts
        self.joint_goal_publisher.publish( self.joint_goal )
        return


    def execute_callback( self, goal ):
        if self.verbose >= 0:
            print( "GenericFJTAS.execute_callback... n_points = " + str( len( goal.trajectory.points )) + " ... " )
            # print( type( goal ))
        if self.verbose >= 3:
            print( goal )

        success = True
        
        # prepare some stuff for our feedback message(s)
        # 
        all_zeros = np.zeros( self.n_joints )

        self.feedback.header.stamp = rospy.Time.now()
        self.feedback.header.frame_id = rospy.get_name() # "diana_gripper_fjtas"
        self.feedback.joint_names = joint_names 

        self.feedback.desired.positions = copy.deepcopy( all_zeros )
        self.feedback.desired.velocities = copy.deepcopy( all_zeros )
        self.feedback.desired.accelerations = copy.deepcopy( all_zeros )
        self.feedback.desired.time_from_start = rospy.Time.now()

        self.feedback.actual.positions = copy.deepcopy( all_zeros )
        self.feedback.actual.velocities = copy.deepcopy( all_zeros )
        self.feedback.actual.accelerations = copy.deepcopy( all_zeros )
        self.feedback.actual.time_from_start = rospy.Time.now()

        self.feedback.error.positions = copy.deepcopy( all_zeros )
        self.feedback.error.velocities = copy.deepcopy( all_zeros )
        self.feedback.error.accelerations = copy.deepcopy( all_zeros )
        self.feedback.error.time_from_start = rospy.Time.now()


        # prepare current and start/end times of the trajectory
        # 
        t_now   = rospy.Time.now()
        t_start = rospy.Time.now() + goal.trajectory.points[0].time_from_start
        t_end   = rospy.Time.now() + goal.trajectory.points[-1].time_from_start
        print( '... trajectory t_start ' + str( t_start.to_sec() ) + ' t_end ' + str( t_end.to_sec() )
               + ' duration ' + str( (t_end - t_start).to_sec() ))

        last_feedback_ii = -1 # none yet
        ii = 0                # index of current trajectory point
        iimax = len( goal.trajectory.points )
        t_ii = t_start + goal.trajectory.points[ ii ].time_from_start

        # main trajectory execution loop: while ongoing, find the
        # bracketing trajectory points, interpolate between them,
        # and forward to the joint-position controller.
        # 
        while t_now < t_end:

            # check for preemption (user canceled the execution)
            #
            if self.action_server.is_preempt_requested():
                rospy.loginfo( '... gripper action canceled!' )
                self.action_server.set_preempted()
                success = False
                break

            # given the current time, find the next trajectory point (if any)
            # 
            t_now = rospy.Time.now()
            while (ii < iimax-1) and (t_ii <= t_now):
                ii = ii + 1
                t_ii = t_start + goal.trajectory.points[ ii ].time_from_start

            i1 = ii               # index of current trajectory point
            i0 = max( 0, ii-1 )   # index of previous trajectory point
            t1 = t_start + goal.trajectory.points[ i1 ].time_from_start
            t0 = t_start + goal.trajectory.points[ i0 ].time_from_start
            dt = (t1 - t0).to_sec()

            if dt <= 0: fraction = 1.0
            else:       fraction = (t_now - t0).to_sec() / (t1-t0).to_sec()

            # publish info to the console for the user
            #
            if self.verbose > 2:
                print( '... trajectory t= ' + str( t_now.to_sec() ) 
                     + '  point indicesr=[' + str( i0 ) + ".." + str( i1 ) + ']'
                     + '  frac=' + str( fraction ) )

            # execute the trajectory point: send joint_goals to servos
            # 
            GG,VV = self.interpolate_point( goal.trajectory.points[i0], goal.trajectory.points[i1], fraction )
            self.move_to_interpolated_point( goal, GG, VV ) 
            rospy.sleep( rospy.Duration( self.t_sleep )) 

            # periodically send feedback messages: once per trajectory point
            #
            if ii != last_feedback_ii:
                last_feedback_i = ii
                self.feedback.header.stamp = rospy.Time.now()
                self.feedback.header.frame_id = "diana_gripper_fjtas_feedback"
                self.feedback.joint_names = joint_names 

                self.feedback.desired.positions = GG
                self.feedback.desired.velocities = VV
                self.feedback.desired.accelerations = copy.deepcopy( all_zeros ) 
                self.feedback.desired.effort = copy.deepcopy( all_zeros )
                self.feedback.desired.time_from_start = t_now - t_start

                mutex.acquire()
                self.feedback.actual.positions = self.joint_state.position
                self.feedback.actual.velocities = self.joint_state.velocity
                self.feedback.actual.accelerations = copy.deepcopy( all_zeros )
                self.feedback.actual.effort = self.joint_state.effort
                self.feedback.actual.time_from_start = t_now - t_start
                mutex.release()

                self.feedback.error.positions = self.feedback.desired.positions - self.feedback.actual.positions
                self.feedback.error.velocities = self.feedback.desired.velocities - self.feedback.actual.velocities
                self.feedback.error.accelerations = copy.deepcopy( all_zeros )
                self.feedback.actual.effort = copy.deepcopy( all_zeros )
                self.feedback.error.time_from_start = rospy.Time.now()

                self.action_server.publish_feedback( self.feedback )
                if self.verbose > 3:
                    print( '... published feedback: \n' + str( self.feedback ) + '\n\n' )

        # end of the t_now < t_end loop. Send the last trajectory point (again)
        # to make sure that we arrive at the target...
        #
        print( '... trajectory loop complete, moving to last trajectory point...' )
        GG,VV = self.interpolate_point( goal.trajectory.points[-1], goal.trajectory.points[-1], 1.0 )
        self.move_to_interpolated_point( goal, GG, VV ) 

        print( '... trajectory executed.' )
        if success: 
            # does NOT work...
            # self.result.header = goal.trajectory.header
            # self.result.header.stamp = rospy.Time.now()
            # self.result.status = self.result.LOST

            # self.result.error_code = self.result.GOAL_TOLERANCE_VIOLATED
            # INVALID_GOAL INVALID_JOINTS OLD_HEADER_TIMESTAMP PATH_TOLERANCE_VIOLATED
            self.result.error_code = self.result.SUCCESSFUL
            self.result.error_string = 'ok.'
            # self.result.goal_id = goal.goal_id HOW TO ACCESS the goal_id ????

            print( '%s: Succeeded' % self.action_name )
            self.action_server.set_succeeded(self.result)




        
if __name__ == '__main__':
    rospy.init_node('generic_fjtas', anonymous=False, disable_signals=True ) 
    # rospy.get_name()
    # rospy.get_namespace()

    if rospy.has_param( "~verbose" ):
        verbose = rospy.get_param( "~verbose" )

    action_namespace = rospy.get_name()
    if rospy.has_param( "~action_namespace" ):
        action_namespace = rospy.get_param( "~action_namespace" )

    joint_names = []
    if rospy.has_param( "~joint_names" ):
        raw = rospy.get_param( "~joint_names" )
        joint_names = raw.split()

    if verbose > 5:
        print( "... action_namespace: '" + action_namespace + "'" )
        print( "... joint_names : '" + str( joint_names ) + "'" )

    fjtas = GenericFollowJointTrajectoryAction( action_namespace + "/follow_joint_trajectory",
                                                joint_names )

    #  rospy.spin() # IMPOSSIBLE to control-c this crap under python3, simply continues running 
    while not rospy.is_shutdown():
        rospy.sleep( rospy.Duration( 0.1 ))
    exit( 0 )
