/*
 * test_joint_state_estimator.cpp
 *
 *  Created on: Feb 4, 2014
 *      Author: Francisco Vina
 */

/* Copyright (c) 2014, Francisco Vina, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <control_msgs/JointTrajectoryControllerState.h>


int main(int argc, char **argv) {

	/// initialize ROS, specify name of node
	ros::init(argc, argv, "test_joint_state_estimator");
	ros::NodeHandle n_ = ros::NodeHandle("~");

	ros::Publisher topicPub_JointState_;

	topicPub_JointState_ = n_.advertise<control_msgs::JointTrajectoryControllerState> ("true_state", 1);

	std::vector<double> joint_acc(7, 0.0);
	std::vector<double> joint_vel(7, 0.0);
	std::vector<double> joint_pos(7, 0.0);

	std::vector<std::string> joint_names;
	joint_names.resize(7);

	joint_names[0] = "left_arm_1_joint";
	joint_names[1] = "left_arm_2_joint";
	joint_names[2] = "left_arm_3_joint";
	joint_names[3] = "left_arm_4_joint";
	joint_names[4] = "left_arm_5_joint";
	joint_names[5] = "left_arm_6_joint";
	joint_names[6] = "left_arm_7_joint";

	control_msgs::JointTrajectoryControllerState controller_state_msg;
	controller_state_msg.joint_names = joint_names;

	double frequency = 100.0;
	double signal_period = 7.0;

	ros::Rate loop_rate(frequency);
	double delta_t = 1/frequency;
	ros::Time begin = ros::Time::now();
	while(n_.ok())
	{

		// trapezoid profile
		double t = (ros::Time::now()-begin).toSec();
		t = fmod(t,signal_period);

		if((t>1.0 && t<2.0)||(t>6.0 && t<7.0))
		{
			for(unsigned int i=0; i<7; i++) joint_acc[i] = 10.0;
		}

		else if(t>3.0 && t<5.0)
		{
			for(unsigned int i=0; i<7; i++) joint_acc[i] = -10.0;
		}

		else
		{
			for(unsigned int i=0; i<7; i++) joint_acc[i] = 0.0;
		}

		// calculate velocity and position

		for(unsigned int i=0; i<7; i++) joint_vel[i] += joint_acc[i]*delta_t;
		for(unsigned int i=0; i<7; i++) joint_pos[i] += joint_vel[i]*delta_t + joint_acc[i]*delta_t*delta_t*0.5;


		controller_state_msg.header.stamp = ros::Time::now();
		controller_state_msg.actual.positions = joint_pos;
		controller_state_msg.actual.velocities = joint_vel;
		controller_state_msg.actual.accelerations = joint_acc;

		topicPub_JointState_.publish(controller_state_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
