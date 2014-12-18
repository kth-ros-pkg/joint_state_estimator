/*
 * JointStateEstimator.cpp
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


JointStateEstimator::JointStateEstimator() {

	m_sys_pdf = NULL;
	m_sys_model = NULL;
	m_prior = NULL;
	m_filter = NULL;
	m_joint_pos_meas_pdf = NULL;
	m_joint_pos_meas_model = NULL;

	m_initialized = false;

}

JointStateEstimator::~JointStateEstimator() {
	delete m_sys_model;
	delete m_sys_pdf;
	delete m_prior;
	delete m_filter;
	delete m_joint_pos_meas_pdf;
	delete m_joint_pos_meas_model;
}

bool JointStateEstimator::init(
		const double &pos_meas_noise_var,
		const std::vector<double> &process_noise_var,
		const std::vector<double> &prior_mu,
		const std::vector<double> &prior_var,
		const double &delta_t)
{

    if(m_initialized)
    {
        ROS_ERROR("Already initialized");
        return false;
    }

	if(prior_mu.size()!=3 || prior_var.size()!=3)
	{
		ROS_ERROR("Wrong prior size");
		return false;
	}


	if(pos_meas_noise_var<=0.0)
	{
		ROS_ERROR("Invalid pos_meas_noise_var");
		return false;
	}

	if(process_noise_var.size()!=3)
	{
		ROS_ERROR("Invalid process_noise_var");
		return false;
	}

	for(unsigned int i=0; i<process_noise_var.size(); i++)
	{
		if(process_noise_var[i]<0.0)
		{
			ROS_ERROR("Invalid process_noise_var");
			return false;
		}
	}


	if(delta_t <= 0.0)
	{
		ROS_ERROR("delta_t <= 0.0");
		return false;
	}


	// set the process model

	MatrixWrapper::Matrix A(3,3);
	for (unsigned int i = 0; i < 3; i++)
	{
		for (unsigned int j = 0; j < 3; j++)
		{
			if (i == j) {
				A(i + 1, j + 1) = 1.0;
			}

			else {
				A(i + 1, j + 1) = 0.0;
			}
		}
	}

	A(1,2) = delta_t;
	A(2,3) = delta_t;

	std::vector<double> sys_mu(3, 0.0);
	std::vector<double> sys_var(3, 0.0);
	sys_var = process_noise_var;

	BFL::Gaussian sys_uncertainty = gaussianDiagonalCov(sys_mu, sys_var);
	delete m_sys_pdf;
	m_sys_pdf = new BFL::LinearAnalyticConditionalGaussian(A, sys_uncertainty);

	delete m_sys_model;
	m_sys_model = new BFL::LinearAnalyticSystemModelGaussianUncertainty(m_sys_pdf);


	// set the LTI measurement model
	MatrixWrapper::Matrix H(1,3);
	H = 0.0;
	H(1,1) = 1.0;
	H(1,2) = 0.0;
	H(1,3) = 0.0;

	MatrixWrapper::ColumnVector joint_pos_meas_mu(1);
	joint_pos_meas_mu(1) = 0.0;

	MatrixWrapper::SymmetricMatrix joint_pos_meas_cov(1);
	joint_pos_meas_cov = 0.0;
	joint_pos_meas_cov(1,1) = pos_meas_noise_var;

	BFL::Gaussian joint_pos_meas_uncertainty(joint_pos_meas_mu, joint_pos_meas_cov);

	delete m_joint_pos_meas_pdf;
	m_joint_pos_meas_pdf = new BFL::LinearAnalyticConditionalGaussian(H, joint_pos_meas_uncertainty);

	delete m_joint_pos_meas_model;
	m_joint_pos_meas_model = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(m_joint_pos_meas_pdf);



	// set the prior
	BFL::Gaussian prior_temp = gaussianDiagonalCov(prior_mu, prior_var);
	delete m_prior;
	m_prior = new BFL::Gaussian(prior_temp.ExpectedValueGet(), prior_temp.CovarianceGet());

	// construct the filter
	delete m_filter;
	m_filter = new BFL::ExtendedKalmanFilter(m_prior);

	m_initialized = true;
	return true;
}

bool JointStateEstimator::update(double joint_pos)
{

	if(!m_initialized)
	{
		ROS_ERROR("Filter not initialized");
		return false;
	}


	MatrixWrapper::ColumnVector joint_pos_meas(1);
	joint_pos_meas(1) = joint_pos;


	m_filter->Update(m_sys_model, m_joint_pos_meas_model, joint_pos_meas);

	return true;
}

double JointStateEstimator::getFilteredJointPos()
{
	MatrixWrapper::ColumnVector posterior_mean = m_filter->PostGet()->ExpectedValueGet();
	double filtered_joint_pos  = posterior_mean(1);

	return filtered_joint_pos;
}

double JointStateEstimator::getFilteredJointVel()
{
	MatrixWrapper::ColumnVector posterior_mean = m_filter->PostGet()->ExpectedValueGet();
	double filtered_joint_vel  = posterior_mean(2);

	return filtered_joint_vel;
}

double JointStateEstimator::getFilteredJointAcc()
{
	MatrixWrapper::ColumnVector posterior_mean = m_filter->PostGet()->ExpectedValueGet();
	double filtered_joint_acc  = posterior_mean(3);

	return filtered_joint_acc;
}

bool JointStateEstimator::reset(const std::vector<double> &prior_mu, const std::vector<double> &prior_var)
{
    if(!m_initialized)
    {
        ROS_ERROR("Not initialized, cannot reset filter");
        return false;
    }

    // set the prior
    BFL::Gaussian prior_temp = gaussianDiagonalCov(prior_mu, prior_var);

    *m_prior = prior_temp;

    m_filter->Reset(m_prior);

}

bool JointStateEstimator::reset()
{
    if(!m_initialized)
    {
        ROS_ERROR("Not initialized, cannot reset filter");
        return false;
    }

    m_filter->Reset(m_prior);
}


BFL::Gaussian JointStateEstimator::gaussianDiagonalCov(const std::vector<double> &mu, const std::vector<double> &var) {

	MatrixWrapper::ColumnVector Mu(mu.size());

	for (unsigned int i = 0; i < mu.size(); i++) {
		Mu(i + 1) = mu[i];
	}

	MatrixWrapper::SymmetricMatrix Cov(var.size());
	Cov = 0.0;
	for (unsigned int i = 0; i < var.size(); i++) {
		for (unsigned int j = 0; j < var.size(); j++) {
			if (i == j) {
				Cov(i + 1, j + 1) = var[i];
			}

			else {
				Cov(i + 1, j + 1) = 0.0;
			}
		}
	}

	return BFL::Gaussian(Mu, Cov);
}
