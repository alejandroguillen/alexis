/*
 * TxSpeedEstimator.h
 *
 *  Created on: Oct 24, 2014
 *      Author: jordi
 */

#ifndef TXSPEEDESTIMATOR_H_
#define TXSPEEDESTIMATOR_H_

#define TX_EXP_COEF_DEFAULT 0.20
#define TX_TRAINING_PERIOD 10

class TxSpeedEstimator {
public:
	TxSpeedEstimator();
	TxSpeedEstimator(float tx_exp_coef);
	float getBandwidth();
	void AddObservation(float txtime, int Npixels);
private:
	float bandwidth;
	int training_samples;
	float tx_exp_coef_;
};

#endif /* TXSPEEDESTIMATOR_H_ */
