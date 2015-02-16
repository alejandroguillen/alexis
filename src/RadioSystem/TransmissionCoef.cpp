/*
 * TransmissionCoef.cpp
 *
 *  Created on: 26/jan/2015
 *      Author: Alejandro Guillen
 */

#include "TransmissionCoef.h"

TransmissionCoef::TransmissionCoef() {
	Ctcoef = 0;
	tx_exp_coef_ = TX_EXP_COEF_DEFAULT;
	training_samples=0;
}

TransmissionCoef::TransmissionCoef(float tx_exp_coef) {
	Ctcoef = 0;
	tx_exp_coef_ = tx_exp_coef;
	training_samples=0;
}

double TransmissionCoef::getTransmissionTimeCoef() {
	return Ctcoef;
}

void TransmissionCoef::AddObservation(double txtime, int Npixels) {
	double transmissiontcoeff= txtime/Npixels; // seconds/pixel (seconds/image)

	//EXPONENTIAL SMOOTH APPLIED: due to Ctcoef can not change too much during the connection (not change position)
	//but before settle down, can change due to the different assignments of the cameras
	if(training_samples <= TX_TRAINING_PERIOD){ //Training period: Arithmetic smoothing
		training_samples++;
		Ctcoef = ((training_samples-1)*Ctcoef + transmissiontcoeff)/training_samples;
	}else{ // Exponential smoothing
		//Ctcoef = (1-tx_exp_coef_)*Ctcoef + tx_exp_coef_*transmissiontcoeff;
		Ctcoef = transmissiontcoeff;
	}

}
