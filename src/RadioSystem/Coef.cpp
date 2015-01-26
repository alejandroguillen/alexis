/*
 * Coef.cpp
 *
 *  Created on: 26/jan/2015
 *      Author: Alejandro Guillen
 */

#include "Coef.h"

#include <iostream>
#include <fstream>

Coef::Coef() {
	Ptcoef = 0;
	Pt_exp_coef_ = PT_EXP_COEF_DEFAULT;
	pt_samples=0;
}

Coef::Coef(float pt_exp_coef) {
	Ptcoef = 0;
	Pt_exp_coef_ = pt_exp_coef;
	pt_samples=0;
}

float Coef::getProcessingTime() {
	return Ptcoef;
}

void Coef::AddObservation(float processingTime, int Npixels, int Nip) {
	int alphad = 5;
	float processingtcoeff = processingTime/(Npixels + alphad*Nip); // seconds/bits (seconds/image)
	if(pt_samples <= PT_TRAINING_PERIOD){ //Training period: Arithmetic smoothing
		pt_samples++;
		Ptcoef = ((pt_samples-1)*Ptcoef + processingtcoeff)/pt_samples;
	}else{ // Exponential smoothing
		Ptcoef = (1-Pt_exp_coef_)*Ptcoef + Pt_exp_coef_*processingtcoeff;
	}
	std::ofstream out("Ptcoef.txt");
	out << Ptcoef << std::endl;
	out.close();
}
