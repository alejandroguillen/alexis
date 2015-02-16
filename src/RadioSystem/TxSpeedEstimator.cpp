/*
 * TxSpeedEstimator.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: jordi
 */

#include "TxSpeedEstimator.h"

TxSpeedEstimator::TxSpeedEstimator() {
	bandwidth = 0;
	tx_exp_coef_ = TX_EXP_COEF_DEFAULT;
	training_samples=0;
}

TxSpeedEstimator::TxSpeedEstimator(float tx_exp_coef) {
	bandwidth = 0;
	tx_exp_coef_ = tx_exp_coef;
	training_samples=0;
}

float TxSpeedEstimator::getBandwidth() {
	return bandwidth;
}

void TxSpeedEstimator::AddObservation(float txtime, int Npixels) {
	float bw = 8*Npixels/txtime; // bits/second
	if(training_samples <= TX_TRAINING_PERIOD){ //Training period: Arithmetic smoothing
		training_samples++;
		bandwidth = ((training_samples-1)*bandwidth + bw)/training_samples;
	}else{ // Exponential smoothing
		bandwidth = (1-tx_exp_coef_)*bandwidth + tx_exp_coef_*bw;
	}
}
