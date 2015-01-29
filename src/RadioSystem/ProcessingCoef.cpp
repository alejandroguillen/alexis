/*
 * ProcessingCoef.cpp
 *
 *  Created on: 26/jan/2015
 *      Author: Alejandro Guillen
 */

#include "ProcessingCoef.h"

ProcessingCoef::ProcessingCoef() {
	Ptcoef = 0;
	Pt_exp_coef_ = PT_EXP_COEF_DEFAULT;
	pt_samples=0;
}

ProcessingCoef::ProcessingCoef(float pt_exp_coef) {
	Ptcoef = 0;
	Pt_exp_coef_ = pt_exp_coef;
	pt_samples=0;
}

float ProcessingCoef::getProcessingTimeCoef() {
	return Ptcoef;
}

void ProcessingCoef::AddObservation(float processingTime, int Npixels, int Nip, double alphad) {
	float processingtcoeff = processingTime/(Npixels + alphad*Nip); // seconds/bits (seconds/image)
	if(pt_samples <= PT_TRAINING_PERIOD){ //Training period: Arithmetic smoothing
		pt_samples++;
		Ptcoef = ((pt_samples-1)*Ptcoef + processingtcoeff)/pt_samples;
	}else{ // Exponential smoothing
		Ptcoef = (1-Pt_exp_coef_)*Ptcoef + Pt_exp_coef_*processingtcoeff;
	}
	//Ptcoef = processingtcoeff;
		/*std::ofstream out("Ptcoef.txt");
		std::streambuf *cerrbuf = std::cerr.rdbuf();
		std::cerr.rdbuf(out.rdbuf());
		std::string word;
		std::cerr << word << " ";
		std::cerr << Ptcoef << "\n";
		std::cerr.rdbuf(cerrbuf);
		out.close();
		std::cerr << word;
		*/
}
