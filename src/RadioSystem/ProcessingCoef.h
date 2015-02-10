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
	float getProcessingTimeCoef();
	float setAlphad(float Pdpx, float Pdip, float Pe, float Pm);
	float getAlphad();
	void AddObservation(float processingtime, int Npixels, int Nip, double alphad);
	void setOverlap(double overlap_normalized, bool double_overlap);
private:
	float Ptcoef;
	int pt_samples;
	float Pt_exp_coef_;
	float alpha_d_;
	double overlapNpixels;
};

#endif /* PROCESSINGCOEF_H_ */
