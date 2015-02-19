/*
 * ProcessingCoef.h
 *
 *  Created on: 26/jan/2015
 *      Author: Alejandro Guillen
 */

#ifndef PROCESSINGCOEF_H_
#define PROCESSINGCOEF_H_

#define PT_EXP_COEF_DEFAULT 0.10
#define PT_TRAINING_PERIOD 20

class ProcessingCoef {
public:
	ProcessingCoef();
	ProcessingCoef(float Pt_exp_coef);
	double getProcessingTimeCoef();
	double setAlphad(double Pdpx, double Pdip, double Pe, double Pm);
	double getAlphad();
	void AddObservation(double processingtime, int Npixels, int Nip, double alphad);
	void setOverlap(double overlap_normalized, bool double_overlap);
private:
	double Ptcoef;
	int pt_samples;
	float Pt_exp_coef_;
	double alpha_d_;
	double overlapNpixels;
};

#endif
