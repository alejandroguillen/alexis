/*
 * ProcessingSpeedEstimator.h
 *
 *  Created on: Oct 8, 2014
 *      Author: jordi
 */

#ifndef PROCESSINGSPEEDESTIMATOR_H_
#define PROCESSINGSPEEDESTIMATOR_H_

#include <vector>

#define NUM_PROCESSING_OBSERVATIONS_DEFAULT 50
#define PE_EXP_COEF_DEFAULT 0.1
#define PE_TRAINING_PERIOD 20
#define PM_EXP_COEF_DEFAULT 0.1
#define PM_TRAINING_PERIOD 20

typedef struct ProcessingPerformanceObservation_t{
	float Tdetect;
	int Npixels;
	int Nip;
}ProcessingPerformanceObservation_t;

class ProcessingSpeedEstimator {
public:
	ProcessingSpeedEstimator();
	ProcessingSpeedEstimator(int num_processing_observations, float Pe_exp_coef, float Pm_exp_coef);
	int AddObservation(float Tdetect, float Textract, float Tmerge, int Npixels, int Nip);
	float getPdpx();
	float getPdip();
	float getPe();
	float getPm();

private:
	float Pdpx_;
	float Pdip_;
	float Pe_;
	float Pm_;
	std::vector<ProcessingPerformanceObservation_t> ProcessObs_;
	int ObsPosition;
	int pe_samples;
	int pm_samples;

	//Configs:
	int num_processing_observations_;
	float Pe_exp_coef_;
	float Pm_exp_coef_;
};

#endif /* PROCESSINGSPEEDESTIMATOR_H_ */
