#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
	
	print ("Matrices constructing!")

        ### Your FK code here
        # Create symbols
	#
	a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")
	d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")
	q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8")
	#
	# Create Modified DH parameters
	#
	print ("41!")

	s = {
		a0: 0,		alpha0: 0.0, 		d1: 0.75,
		a1: 0.35,	alpha1: -0.5 * pi,	d2: 0.0,
		a2: 1.25,	alpha2: 0.0,		d3: 0.0,
		a3: -0.054,	alpha3: -0.5 * pi,	d4: 1.5,
		a4: 0.0,	alpha4: +0.5 * pi,	d5: 0.0,
		a5: 0.0,	alpha5: -0.5 * pi,	d6: 0.0,
		a6: 0.0,	alpha6: 0.0,		d7: 0.193, 	q7: 0
	}
	#
	# Define Modified DH Transformation matrix
	#
	T0_1 = Matrix([
		[            cos(q1),	           -sin(q1), 		   0, 		     a0],
		[sin(q1)*cos(alpha0),	cos(q1)*cos(alpha0),	-sin(alpha0),	-sin(alpha0)*d1],
		[sin(q1)*sin(alpha0),	cos(q1)*sin(alpha0),	 cos(alpha0),	 cos(alpha0)*d1],
		[                  0, 			  0,               0, 		      1]])
	T0_1.subs(s)

	T1_2 = Matrix([
		[            cos(q2),	           -sin(q2), 		   0, 		     a1],
		[sin(q2)*cos(alpha1),	cos(q2)*cos(alpha1),	-sin(alpha1),	-sin(alpha1)*d2],
		[sin(q2)*sin(alpha1),	cos(q2)*sin(alpha1),	 cos(alpha1),	 cos(alpha1)*d2],
		[                  0, 			  0,               0, 		      1]])
	T1_2.subs(s)

	T2_3 = Matrix([
		[            cos(q3),	           -sin(q3), 		   0, 		     a2],
		[sin(q3)*cos(alpha2),	cos(q3)*cos(alpha2),	-sin(alpha2),	-sin(alpha2)*d3],
		[sin(q3)*sin(alpha2),	cos(q3)*sin(alpha2),	 cos(alpha2),	 cos(alpha2)*d3],
		[                  0, 			  0,               0, 		      1]])
	T2_3.subs(s)

	print ("102!")

	#
	# Create individual transformation matrices
	#
	T0_2 = simplify(T0_1 * T1_2)
	T0_3 = simplify(T0_2 * T2_3)

	print ("116!")
	#
	# Extract rotation matrices from the transformation matrices
	#

	R_int_z = Matrix([
		[         cos(pi),	-sin(pi),		0],
		[	  sin(pi),	 cos(pi),		0],
		[		0,	       0,		1]])

	R_int_y = Matrix([
		[    cos(-0.5*pi),		0,	sin(-0.5*pi)],
		[	  	0,		1,		   0],
		[   -sin(-0.5*pi),		0, 	cos(-0.5*pi)]])

	print("130")
	R_corr = R_int_z * R_int_y
	#
        ###

	print ("Matrices constructed!")

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    print("160")
	    R_ext_z = Matrix([
		[	cos(yaw),	-sin(yaw), 	0],
		[	sin(yaw),	 cos(yaw),	0],
		[	       0,	        0,	1]])

	    R_ext_y = Matrix([
		[     cos(pitch),	0, sin(pitch)],
		[	       0,	1,	    0],
		[    -sin(pitch),	0, cos(pitch)]])

	    R_ext_x = Matrix([
		[       1, 		0, 		 0],
		[	0,	cos(roll),	-sin(roll)],
		[	0,	sin(roll),	 cos(roll)]])
	    print("175")

	    R_ext = R_ext_z * R_ext_y * R_ext_x
	    R0_6 = R_ext * R_corr

	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

	    print("183")
	    
	    d_7 = s[d7]
            wc_x = px - d_7 * R0_6[0, 2]
            wc_y = py - d_7 * R0_6[1, 2]
            wc_z = pz - d_7 * R0_6[2, 2]


            theta1 = (atan2(wc_y, wc_x)).evalf()
	    print("194")
	    a_1 = s[a1]
	    a_2 = s[a2]
	    a_3 = s[a3]
	    d_1 = s[d1]
	    d_4 = s[d4]
	    print("200")
	    s1 = sqrt(a_3**2 + d_4**2) 	#sqrt( (-0.054)^2 + (1.5)^2 )
	    s2 = sqrt(wc_x**2 + wc_y**2) - a_1
	    s3 = wc_z - d_1		#wc_z - 0.75
            ss = sqrt(s2**2 + s3**2)

	    D1 = (ss**2 + a_2**2 - s1**2) / (2*ss*a_2)
            beta1 = atan2(s3, s2)
	    beta2 = atan2(sqrt(1 - D1**2), D1)
            theta2 = (0.5 * pi - beta1 - beta2).evalf()

	    D2 = (s1**2+a_2**2-ss**2) / (2*s1*a_2)
            gamma1 = atan2(a_3, d_4)
	    gamma2 = atan2(sqrt(1 - D2**2), D2)
	    theta3 = (0.5 * pi - gamma1 - gamma2).evalf()

	    print("216")
	    #theta1 = theta1.evalf()
	    #theta2 = theta2.evalf()
	    #theta3 = theta3.evalf()
	    print("220")

	    s.update({q1: theta1, q2: theta2 - 0.5 * pi.evalf(), q3: theta3})	# append the new values
	    print(s)
	    R0_3 = T0_3.evalf(subs=s)
	    print("222")
	    R0_3 = R0_3[:3, :3]
	    R0_3_inv = R0_3.inv("LU")
	    print("226")
            R3_6 = simplify(R0_3_inv * R0_6)
	    print("227")
	
	    r11 = R3_6[0, 0]
	    r12 = R3_6[0, 1]
	    r13 = R3_6[0, 2]

	    r21 = R3_6[1, 0]
	    r22 = R3_6[1, 1]
	    r23 = R3_6[1, 2]

	    r31 = R3_6[2, 0]
	    r32 = R3_6[2, 1]
	    r33 = R3_6[2, 2]

            theta4 = (atan2(r33, -r13)).evalf()
            theta5 = (atan2(sqrt(r13**2 + r33**2), r23)).evalf()
	    theta6 = (atan2(-r22, r21)).evalf()
	    print("244")
	    #theta4 = theta4.evalf()
	    #theta5 = theta5.evalf()
	    #theta6 = theta6.evalf()
	    
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            #theta1 = theta2 = theta3 = theta4 = theta5 = theta6 = pi / (x+1)
	    print("wc_x: " + str(wc_x) + ", wc_y: " + str(wc_y) + ", wc_z: " + str(wc_z))
	    print("t1: " + str(theta1) + ", t2: " + str(theta2) + ", t3: " + str(theta3))
	    print("t4: " + str(theta4) + ", t5: " + str(theta5) + ", t6: " + str(theta6))
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

	    print("Service was invoked! Iteration No. " + str(x))

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
