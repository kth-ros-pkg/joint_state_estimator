/*
 * joint_state_estimator_node.cpp
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
#include <joint_state_estimator/JointStateEstimator.h>
#include <control_msgs/JointTrajectoryControllerState.h>



class JointStateEstimatorNode{

public:
	/// create a handle for this node, initialize node
	ros::NodeHandle n_;

	/// declaration of topics to publish
	ros::Publisher topicPub_EstimatedJointStates_;

	/// declaration of topics to subscribe, callback is called for new messages arriving
	ros::Subscriber topicSub_JointState_;


	JointStateEstimatorNode()
	{
		n_ = ros::NodeHandle("~");

		topicPub_EstimatedJointStates_ = n_.advertise<control_msgs::JointTrajectoryControllerState>("estimated_joint_state", 1);
		topicSub_JointState_ = n_.subscribe("state", 1, &JointStateEstimatorNode::topicCallback_JointStates, this);

		m_last_filter_update = ros::Time::now();

		m_filters_initialized = false;
	}

	~JointStateEstimatorNode()
	{
		for(unsigned int i=0; i<m_filters.size(); i++)
			delete m_filters[i];
	}

	bool getROSParameters()
	{

		/// Get joint names
		XmlRpc::XmlRpcValue JointNamesXmlRpc;
		std::vector<std::string> JointNames;
		if (n_.hasParam("joint_names"))
		{
			n_.getParam("joint_names", JointNamesXmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter joint_names not set, shutting down node...");
			n_.shutdown();
			return false;
		}


		/// Resize and assign of values to the JointNames
		JointNames.resize(JointNamesXmlRpc.size());
		for (unsigned int i = 0; i < JointNamesXmlRpc.size(); i++)
		{
			JointNames[i] = (std::string)JointNamesXmlRpc[i];
		}


		/// get joint position measurement variance
		double pos_meas_noise_var;
		if (n_.hasParam("pos_meas_noise_var"))
		{
			n_.getParam("pos_meas_noise_var", pos_meas_noise_var);
		}

		else
		{
			ROS_ERROR("Parameter pos_meas_noise_var not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		/// get joint acceleration process variance
		XmlRpc::XmlRpcValue ProcessNoiseVarXmlRpc;
		std::vector<double> process_noise_var;
		if (n_.hasParam("process_noise_var"))
		{
			n_.getParam("process_noise_var", ProcessNoiseVarXmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter process_noise_var not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if(ProcessNoiseVarXmlRpc.size()!=3)
		{
			ROS_ERROR("Invalid process_noise_var size");
			return false;
		}

		// Resize and assign of values to the JointNames
		process_noise_var.resize(ProcessNoiseVarXmlRpc.size());
		for (unsigned int i = 0; i < ProcessNoiseVarXmlRpc.size(); i++)
		{
			process_noise_var[i] = (double)ProcessNoiseVarXmlRpc[i];
		}

		/// Get prior variance
		XmlRpc::XmlRpcValue PriorVarXmlRpc;
		std::vector<double> prior_var;
		if (n_.hasParam("prior_var"))
		{
			n_.getParam("prior_var", PriorVarXmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter joint_names not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if(PriorVarXmlRpc.size()!= 3)
		{
			ROS_ERROR("Incorrect prior var size");
			n_.shutdown();
			return false;
		}

		/// Resize and assign of values to prior_var
		prior_var.resize(PriorVarXmlRpc.size());
		for (int i = 0; i < PriorVarXmlRpc.size(); i++)
		{
			prior_var[i] = (double)PriorVarXmlRpc[i];
		}

		double delta_t;
		if (n_.hasParam("delta_t"))
		{
			n_.getParam("delta_t", delta_t);
		}

		else
		{
			ROS_ERROR("Parameter delta_t not set, shutting down node...");
			n_.shutdown();
			return false;
		}


		m_joint_names = JointNames;
		m_DOF = m_joint_names.size();
		m_pos_meas_noise_var = pos_meas_noise_var;
		m_process_noise_var = process_noise_var;
		m_prior_var = prior_var;
		m_delta_t = delta_t;

		m_joint_pos.resize(m_DOF);
		m_filtered_joint_pos.resize(m_DOF);
		m_filtered_joint_vel.resize(m_DOF);
		m_filtered_joint_acc.resize(m_DOF);
		m_filters.resize(m_DOF);

		return true;
	}

	bool InitFilters()
	{

		ROS_DEBUG("Initializing filters");
		std::vector<double> prior_mu(3, 0.0);

		for(unsigned int i=0; i<m_DOF; i++)
		{
			delete m_filters[i];
			m_filters[i] = new JointStateEstimator();
			prior_mu[0] = m_joint_pos[i];
			if(!m_filters[i]->init(m_pos_meas_noise_var, m_process_noise_var, prior_mu, m_prior_var, m_delta_t))
			{
				return false;
			}
		}

		m_filters_initialized = true;
		ROS_DEBUG("Filters initialized");
		return true;
	}

	void topicCallback_JointStates(const control_msgs::JointTrajectoryControllerStatePtr &msg)
	{
		ROS_DEBUG("Received joint states");

		// search for joints in joint state msg
		if(msg->actual.positions.size()!=m_DOF)
		{
			ROS_ERROR("Invalid joint states size received");
			return;
		}

		for(unsigned int i=0; i<m_DOF; i++)
		{
			if(msg->joint_names[i]!=m_joint_names[i])
			{
				ROS_ERROR("Error in received joint name");
				return;
			}

			m_joint_pos[i] = msg->actual.positions[i];
		}

		if(!FiltersInitialized())
		{
			InitFilters();
		}

		UpdateFilters();
		PublishEstimatedJointStates();

	}

	void UpdateFilters()
	{
		if(!FiltersInitialized())
		{
			ROS_ERROR("Filters not initialized");
			return;
		}

		ROS_DEBUG("Updating filters");

//		double delta_t = (ros::Time::now()-m_last_filter_update).toSec();
//		delta_t = 0.01;
		m_last_filter_update = ros::Time::now();

		for(unsigned int i=0; i<m_DOF; i++)
		{
			m_filters[i]->update(m_joint_pos[i]);
			m_filtered_joint_pos[i] = m_filters[i]->getFilteredJointPos();
			m_filtered_joint_vel[i] = m_filters[i]->getFilteredJointVel();
			m_filtered_joint_acc[i] = m_filters[i]->getFilteredJointAcc();
		}
	}

	void PublishEstimatedJointStates()
	{
		if(!FiltersInitialized())
		{
			return;
		}

		control_msgs::JointTrajectoryControllerState estimated_joint_state_msg;
		estimated_joint_state_msg.header.stamp = ros::Time::now();
		estimated_joint_state_msg.joint_names = m_joint_names;
		estimated_joint_state_msg.actual.positions = m_filtered_joint_pos;
		estimated_joint_state_msg.actual.velocities = m_filtered_joint_vel;
		estimated_joint_state_msg.actual.accelerations = m_filtered_joint_acc;

		topicPub_EstimatedJointStates_.publish(estimated_joint_state_msg);
	}

	bool FiltersInitialized()
	{
		return m_filters_initialized;
	}



private:
	unsigned int m_DOF;
	std::vector<std::string> m_joint_names;

	double m_pos_meas_noise_var;
	std::vector<double> m_process_noise_var;

	std::vector<double> m_prior_var;

	double m_delta_t;

	std::vector<JointStateEstimator*> m_filters;


	std::vector<double> m_joint_pos;
	std::vector<double> m_filtered_joint_pos;
	std::vector<double> m_filtered_joint_vel;
	std::vector<double> m_filtered_joint_acc;

	ros::Time m_last_filter_update;

	bool m_filters_initialized;
};

int main(int argc, char **argv) {

	/// initialize ROS, specify name of node
	ros::init(argc, argv, "joint_state_estimator");

	JointStateEstimatorNode joint_state_estimator_node;
	if(!joint_state_estimator_node.getROSParameters())
	{
		return -1;
	}

	double loop_frequency;
	if(joint_state_estimator_node.n_.hasParam("loop_frequency"))
	{
		joint_state_estimator_node.n_.getParam("loop_frequency", loop_frequency);
	}

	ros::Rate loop_rate(loop_frequency);

	while(joint_state_estimator_node.n_.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}


}

