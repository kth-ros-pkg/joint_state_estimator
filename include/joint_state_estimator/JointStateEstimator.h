/*
 * JointStateEstimator.h
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

#ifndef JOINTSTATEESTIMATOR_H_
#define JOINTSTATEESTIMATOR_H_

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>


/* Estimates velocity and acceleration of a joint given
joint position measurements from an encoder */

class JointStateEstimator {
public:
	JointStateEstimator();
	virtual ~JointStateEstimator();

	// prior of size 3 [pos vel acc]
	bool init(
			const double &pos_meas_noise_var,
			const std::vector<double> &process_noise_var,
			const std::vector<double> &prior_mu,
			const std::vector<double> &prior_var,
			const double &delta_t);

	bool update(double joint_pos);
	double getFilteredJointPos();
	double getFilteredJointVel();
	double getFilteredJointAcc();

    bool reset(const std::vector<double> &prior_mu,
               const std::vector<double> &prior_var);

    bool reset();

	BFL::Gaussian gaussianDiagonalCov(const std::vector<double> &mu, const std::vector<double> &var);

private:


	bool m_initialized;

	BFL::LinearAnalyticConditionalGaussian* m_sys_pdf;
	BFL::LinearAnalyticSystemModelGaussianUncertainty* m_sys_model;

	BFL::LinearAnalyticConditionalGaussian *m_joint_pos_meas_pdf;
	BFL::LinearAnalyticMeasurementModelGaussianUncertainty *m_joint_pos_meas_model;

	BFL::Gaussian *m_prior;
	BFL::ExtendedKalmanFilter *m_filter;

	double m_filtered_joint_vel;
	double m_filtered_joint_acc;

};

#endif /* JOINTSTATEESTIMATOR_H_ */
