/*
 * ProcessingSpeedEstimator.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: jordi
 */

#include "ProcessingSpeedEstimator.h"

#include <iostream>
#include <fstream>

#include <vector>
#include <gsl/gsl_multifit.h>

ProcessingSpeedEstimator::ProcessingSpeedEstimator() {
	num_processing_observations_=NUM_PROCESSING_OBSERVATIONS_DEFAULT;
	ObsPosition=0;
	Pe_ = 0;
	Pm_ = 0;
	Pe_exp_coef_ = PE_EXP_COEF_DEFAULT;
	Pm_exp_coef_ = PM_EXP_COEF_DEFAULT;
	pe_samples=0;
	pm_samples=0;
}

ProcessingSpeedEstimator::ProcessingSpeedEstimator(
		int num_processing_observations, float Pe_exp_coef, float Pm_exp_coef) {
	num_processing_observations_=num_processing_observations;
	ObsPosition=0;
	Pe_ = 0;
	Pe_exp_coef_ = Pe_exp_coef;
	Pm_ = 0;
	Pm_exp_coef_ = Pm_exp_coef;
	pe_samples=0;
	pm_samples=0;
}

int ProcessingSpeedEstimator::AddObservation(float Tdetect, float Textract, float Tmerge,
		int Npixels, int Nip) {
	//Estimates the detection rate as a function of the number of pixels (Pdpx)
	// and the detection rate as a function of the number of Interest Points (Pdip)
	// And the extraction rate as a function of the number of interest points (Pe)
	//Both Pdpx and Pdip are calculated from the observation of Tdetect by solving a Least-Squares fit of
	//multiple observations of {Tdetect, Npixels, Nip}.
	//Pe is calculated from Textract. Exponential smoothing is applied.
	//We need at least two samples, with different Npixels and/or Nip to perform the Least-Squares fit
	//If we don't, return -1
	//If the fit is computed, return 0
	//To get the values, getPdpx(), getPdip(), getPe() and getPm().

	//EXPONENTIAL SMOOTH APPLIED: only one task is processed at the time, so these parameters do no change depending on the Nthreads (Ncameras)

	//Pm:
	if(pm_samples <= PM_TRAINING_PERIOD){ //Training period: Arithmetic smoothing
		pm_samples++;
		Pm_ = ((pm_samples-1)*Pm_ + Npixels/Tmerge)/pm_samples;
	}else{ // Exponential smoothing
		Pm_ = (1-Pm_exp_coef_)*Pm_ + Pm_exp_coef_*(Nip/Textract);
	}

	//Pe:
	if(Nip > 0){
		if(pe_samples <= PE_TRAINING_PERIOD){ //Training period: Arithmetic smoothing
			pe_samples++;
			Pe_ = ((pe_samples-1)*Pe_ + Nip/Textract)/pe_samples;
		}else{ // Exponential smoothing
			Pe_ = (1-Pe_exp_coef_)*Pe_ + Pe_exp_coef_*(Nip/Textract);
		}
	}

	//Pdpx and Pdip:
	ProcessingPerformanceObservation_t obs;
	obs.Tdetect=Tdetect;
	obs.Npixels=Npixels;
	obs.Nip=Nip;
	if(ProcessObs_.size()<num_processing_observations_){
		ProcessObs_.push_back(obs);
	}else{
		ProcessObs_.at(ObsPosition)=obs;
	}
	ObsPosition=(ObsPosition+1)%num_processing_observations_;

	if(ProcessObs_.size() < 2){
		return -1;
	}

	int n=(int)ProcessObs_.size();
	gsl_matrix *X, *cov;
	gsl_vector *y,*c;
	double chisq;
	X = gsl_matrix_alloc(n, 2);
  	y = gsl_vector_alloc(n);
	c = gsl_vector_alloc(2);
	cov = gsl_matrix_alloc(2, 2);

	for (int i=0; i<n; i++){
		gsl_matrix_set(X, i, 0, ProcessObs_.at(i).Npixels);
		gsl_matrix_set(X, i, 1, ProcessObs_.at(i).Nip);
		gsl_vector_set(y, i, ProcessObs_.at(i).Tdetect);
	}
/*	Not robust fit
	gsl_multifit_linear_workspace * work = gsl_multifit_linear_alloc(n, 2);
    gsl_multifit_linear(X, y, c, cov, &chisq, work);
    gsl_multifit_linear_free(work);
    */

	//Robust fit:
	gsl_set_error_handler_off();
	gsl_multifit_robust_workspace *work = gsl_multifit_robust_alloc(gsl_multifit_robust_default, n, 2);
	int s = gsl_multifit_robust(X, y, c, cov, work);
	gsl_multifit_robust_free(work);
	if(s==0){
		Pdpx_=1/gsl_vector_get(c,(0));
		Pdip_=1/gsl_vector_get(c,(1));
	}else{
		fprintf(stderr, "ProcessingParameterEstimation: gsl_multifit_robust failed\n");
	}

	gsl_matrix_free(X);
	gsl_vector_free(y);
	gsl_vector_free(c);
	gsl_matrix_free(cov);
	
	//std::ifstream configfile(LB_CONFIG_FILE);

	std::ofstream out("Pdpx-Pdip-Pe-Pm.txt", std::ios::app);

	out << "Num: " << ProcessObs_.size() << "	" << Pdpx_ << "		" << Pdip_ << "		" << Pe_ << "	" << Pm_ << std::endl;

	out.close();
	
	return 0;
}

float ProcessingSpeedEstimator::getPdpx() {
	return Pdpx_;
}

float ProcessingSpeedEstimator::getPdip() {
	return Pdip_;
}

float ProcessingSpeedEstimator::getPe() {
	return Pe_;
}

float ProcessingSpeedEstimator::getPm() {
	return Pm_;
}
