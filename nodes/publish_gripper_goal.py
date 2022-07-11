#! /usr/bin/env python 
# publish_gripper_goal.py - publishes a demo Moveit trajectory goal to the fjtas controller
# 
#

print( "#### 1 ####" )

import rospy
import trajectory_msgs.msg
import control_msgs.msg
import actionlib

print( "#### 2 ####" )

rospy.init_node('diana_gripper_goal_publisher', anonymous=False, disable_signals=True )

pub = rospy.Publisher( '/diana_gripper_controller/follow_joint_trajectory_action/goal',
                       control_msgs.msg.FollowJointTrajectoryActionGoal,
                       queue_size=1 )

print( "#### 3 ####" )

msg = control_msgs.msg.FollowJointTrajectoryActionGoal()
msg.header.seq = 2
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = "HUGO"

msg.goal_id.stamp = msg.header.stamp
msg.goal_id.id = "/move_group-5-1657298927.312892807"

print( "#### 4 ####" )

msg.goal.trajectory.header.stamp = rospy.Time( 0 )
msg.goal.trajectory.joint_names = ["diana_gripper/LFJ1", "diana_gripper/LFJ2", "diana_gripper/RFJ1", "diana_gripper/RFJ2"]
msg.goal.trajectory.points = []
p1 = trajectory_msgs.msg.JointTrajectoryPoint()
p1.positions = [0.0, 0.0, 0.0, 0.0]
p1.velocities = [0.0, 0.0, 0.0, 0.0]
p1.accelerations = [0.0, 0.0, 0.0, 0.0]
p1.effort = []
p1.time_from_start = rospy.Duration( 0.1 )
msg.goal.trajectory.points.append( p1 )

p2 = trajectory_msgs.msg.JointTrajectoryPoint()
p2.positions = [0.25, 0.5, 0.25, -0.5]
p2.velocities =  [0.1, 0.1, 0.1, -0.1]
p2.accelerations = [0.0, 0.0, 0.0, 0.0]
p2.effort = []
p2.time_from_start = rospy.Duration( 1 )
msg.goal.trajectory.points.append( p2 )

p3 = trajectory_msgs.msg.JointTrajectoryPoint()
p3.positions = [0.5, 1.0, 0.5, -1.0]
p3.velocities =  [0.1, 0.1, 0.1, -0.1]
p3.accelerations = [0.0, 0.0, 0.0, 0.0]
p3.effort: []
p3.time_from_start = rospy.Duration( 2 )
msg.goal.trajectory.points.append( p3 )

p4 = trajectory_msgs.msg.JointTrajectoryPoint()
p4.positions = [0.2, 1.0, 0.2, -0.5]
p4.velocities =  [-0.1, 0.0, -0.1, 0.1]
p4.accelerations = [0.0, 0.0, 0.0, 0.0]
p4.effort = []
p4.time_from_start = rospy.Duration( 4 )
msg.goal.trajectory.points.append( p4 )

p5 = trajectory_msgs.msg.JointTrajectoryPoint()
p5.positions = [0.0, -0.3, 0.0, 0.3]
p5.velocities =  [0, 0, 0, 0]
p5.accelerations = [0.0, 0.0, 0.0, 0.0]
p5.effort = []
p5.time_from_start = rospy.Duration( 5 )
msg.goal.trajectory.points.append( p5 )

p6 = trajectory_msgs.msg.JointTrajectoryPoint()
p6.positions = [0.0, 0.0, 0.0, 0.0]
p6.velocities =  [0, 0, 0, 0]
p6.accelerations = [0.0, 0.0, 0.0, 0.0]
p6.effort = []
p6.time_from_start = rospy.Duration( 7 )
msg.goal.trajectory.points.append( p6 )


print( "#### 5 ####" )

pub.publish( msg )
rospy.sleep( rospy.Duration( 1.0 ))
pub.publish( msg )

print( "#### 6 ####" )

"""

        positions: [0.4590090141644092, -0.10735408209462075, 0.4186860913789374, -0.09852254085305888]
        velocities: [0.05, 4.0405984411793055e-06, 0.04311738312591368, -0.00030942022864135506]
        accelerations: [0.0, 3.899822424106114e-21, 4.107515824064223e-17, 5.704883088978087e-18]
        effort: []
        time_from_start: 
          secs: 6
          nsecs:  81539092
      - 
        positions: [0.535028252811399, -0.10734793883027721, 0.4842411041325879, -0.09899297865712474]
        velocities: [0.05, 4.0405984411793055e-06, 0.04311738312591368, -0.00030942022864135506]
        accelerations: [0.0, -3.899822424106114e-21, -4.107515824064223e-17, -5.704883088978087e-18]
        effort: []
        time_from_start: 
          secs: 7
          nsecs: 601923865
      - 
        positions: [0.6110474914583889, -0.10734179556593368, 0.5497961168862384, -0.09946341646119061]
        velocities: [0.05, 4.040598441174742e-06, 0.04311738312591368, -0.0003094202286413596]
        accelerations: [0.0, -5.999598240742678e-18, 4.107515824064223e-17, -2.8524415444890435e-19]
        effort: []
        time_from_start: 
          secs: 9
          nsecs: 122308638
      - 
        positions: [0.6870667301053787, -0.10733565230159016, 0.6153511296398889, -0.09993385426525649]
        velocities: [0.05, 4.040598441174742e-06, 0.04311738312591372, -0.0003094202286413596]
        accelerations: [0.0, 5.999598240742678e-18, 9.12781294236494e-18, 2.8524415444890435e-19]
        effort: []
        time_from_start: 
          secs: 10
          nsecs: 642693411
      - 
        positions: [0.7630859687523686, -0.10732950903724663, 0.6809061423935395, -0.10040429206932236]
        velocities: [0.05, 4.0405984411793055e-06, 0.04311738312591372, -0.00030942022864135506]
        accelerations: [0.0, 3.899822424106114e-21, -9.12781294236494e-18, 5.704883088978087e-18]
        effort: []
        time_from_start: 
          secs: 12
          nsecs: 163078184
      - 
        positions: [0.8391052073993583, -0.10732336577290309, 0.74646115514719, -0.10087472987338822]
        velocities: [0.05, 4.0405984411793055e-06, 0.04311738312591368, -0.00030942022864135506]
        accelerations: [0.0, -3.899822424106114e-21, -4.107515824064223e-17, -5.704883088978087e-18]
        effort: []
        time_from_start: 
          secs: 13
          nsecs: 683462956
      - 
        positions: [0.9151244460463482, -0.10731722250855956, 0.8120161679008405, -0.10134516767745409]
        velocities: [0.05, 4.0405984411793055e-06, 0.04311738312591368, -0.0003094202286413596]
        accelerations: [0.0, 3.899822424106114e-21, 4.107515824064223e-17, -2.8524415444890435e-19]
        effort: []
        time_from_start: 
          secs: 15
          nsecs: 203847729
      - 
        positions: [0.991143684693338, -0.10731107924421603, 0.877571180654491, -0.10181560548151997]
        velocities: [0.05, 4.0405984411793055e-06, 0.04311738312591368, -0.000309420228641355]
        accelerations: [0.0, -3.899822424106114e-21, -4.107515824064223e-17, 6.311026917182009e-18]
        effort: []
        time_from_start: 
          secs: 16
          nsecs: 724232502
      - 
        positions: [1.067162923340328, -0.10730493597987249, 0.9431261934081415, -0.10228604328558583]
        velocities: [0.05, 4.0405984411793055e-06, 0.04311738312591368, -0.000309420228641355]
        accelerations: [0.0, 3.899822424106114e-21, 4.107515824064223e-17, -6.311026917182009e-18]
        effort: []
        time_from_start: 
          secs: 18
          nsecs: 244617275
      - 
        positions: [1.1431821619873177, -0.10729879271552896, 1.008681206161792, -0.1027564810896517]
        velocities: [0.0, 0.0, 0.0, 0.0]
        accelerations: [-0.06577282394550778, -5.315231398123596e-06, -0.056719040986634606, 0.00040702884447213846]
        effort: []
        time_from_start: 
          secs: 19
          nsecs: 765002048
  path_tolerance: []
  goal_tolerance: []
  goal_time_tolerance: 
    secs: 0
    nsecs:         0
"""

