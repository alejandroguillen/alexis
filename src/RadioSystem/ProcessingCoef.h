/*
 * ProcessingCoef.h
 *
 *  Created on: 26/jan/2015
 *      Author: Alejandro Guillen
 */

#ifndef PROCESSINGCOEF_H_
#define PROCESSINGCOEF_H_

#define PT_EXP_COEF_DEFAULT 0.20
#define PT_TRAINING_PERIOD 10

class ProcessingCoef {
public:
	ProcessingCoef();
	ProcessingCoef(float Pt_exp_coef);
	float getProcessingTimeCoef();
	void AddObservation(float processingtime, int Npixels, int Nip, double alphad);
private:
	float Ptcoef;
	int pt_samples;
	float Pt_exp_coef_;
};

#endif /* PROCESSINGCOEF_H_ */
