/*
 * TransmissionCoef.cpp
 *
 *  Created on: 26/jan/2015
 *      Author: Alejandro Guillen
 */

#ifndef TRANSMISSIONCOEF_H_
#define TRANSMISSIONCOEF_H_

#define TX_EXP_COEF_DEFAULT 0.20
#define TX_TRAINING_PERIOD 10

class TransmissionCoef {
public:
	TransmissionCoef();
	TransmissionCoef(float tx_exp_coef);
	float getTransmissionTimeCoef();
	void AddObservation(float txtime, int Npixels);
private:
	double Ctcoef;
	int training_samples;
	float tx_exp_coef_;
};

#endif /* TRANSMISSIONCOEF_H_ */
