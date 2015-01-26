/*
 * Coef.h
 *
 *  Created on: 26/jan/2015
 *      Author: alejandro
 */

#ifndef COEF_H_
#define COEF_H_

#define PT_EXP_COEF_DEFAULT 0.20
#define PT_TRAINING_PERIOD 10

class Coef {
public:
	Coef();
	Coef(float Pt_exp_coef);
	float getProcessingTime();
	void AddObservation(float processingtime, int Npixels, int Nip);
private:
	float Ptcoef;
	int pt_samples;
	float Pt_exp_coef_;
};

#endif /* COEF_H_ */
