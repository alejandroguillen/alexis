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
	alpha_d_=0;
	overlapNpixels=0;
}

ProcessingCoef::ProcessingCoef(float pt_exp_coef) {
	Ptcoef = 0;
	Pt_exp_coef_ = pt_exp_coef;
	pt_samples=0;
	alpha_d_=0;
}

double ProcessingCoef::getProcessingTimeCoef() {
	return Ptcoef;
}

double ProcessingCoef::setAlphad(double Pdpx, double Pdip, double Pe, double Pm){

	//the dividend part of the Pcoef has to be the minimum possible, so alphad->0 => deleting Pm parameter
	//float alpha_d = (Pdpx+Pm)*((1/Pdip) + (1/Pe)); // (pixels/keypoints)
	double alpha_d = (Pdpx)*((1/Pdip) + (1/Pe)); // (pixels/keypoints)
	alpha_d_ = alpha_d;
	return alpha_d;
}

double ProcessingCoef::getAlphad(){
	return alpha_d_;
}

void ProcessingCoef::setOverlap(double overlap_normalized, bool double_overlap){

	overlapNpixels = overlap_normalized;
	if(double_overlap == true){
		overlapNpixels = 2*overlapNpixels;
	}
 }

void ProcessingCoef::AddObservation(double processingTime, int Npixels, int Nip, double alphad) {

	//Npixels = PixelsSlicetoCompute + PixelsOverlap
	double processingtcoeff = processingTime/(Npixels-overlapNpixels + alphad*Nip); // seconds/pixels (seconds/image)
	//we have to calculate all the pixels that arrive at the Coop, because all the subslices (including overlap) are calculated (not to compute though)
	//float processingtcoeff = processingTime/(Npixels + alphad*Nip);

	/*NO EXPONENTIAL SMOOTH APPLIED: the Pcoeff can change due to Nthreads (Ncameras)
	if(pt_samples <= PT_TRAINING_PERIOD){ //Training period: Arithmetic smoothing
		pt_samples++;
		Ptcoef = ((pt_samples-1)*Ptcoef + processingtcoeff)/pt_samples;
	}else{ // Exponential smoothing
		Ptcoef = (1-Pt_exp_coef_)*Ptcoef + Pt_exp_coef_*processingtcoeff;
	}
	*/
	Ptcoef = processingtcoeff;

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
