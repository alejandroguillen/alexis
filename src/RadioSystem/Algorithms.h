/*
 * Algorithms.h
 *
 *  Created on: 26/jan/2015
 *      Author: Alejandro Guillen
 */

#ifndef ALGORITHMS_H_
#define ALGORITHMS_H_

#include "LoadBalancingConfig.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "lp_solve/lp_lib.h"

#define LB_CONFIG_FILE	"./loadbalancing.conf"

#define BDR_UPDATE_COEF_DEFAULT 0.01	//0 for unweighted cumulative moving average, otherwise it is the coefficient for exponential moving average
#define FDR_UPDATE_COEF_DEFAULT 0.01	//0 for unweighted cumulative moving average, otherwise it is the coefficient for exponential moving average
#define SCALING_COEF_DEFAULT	0.95
#define RECONSTRUCTION_METHOD_DEFAULT RECONSTRUCTION_METHOD_BACKWARD
//#define RECONSTRUCTION_METHOD_DEFAULT RECONSTRUCTION_METHOD_SCALING

#define NUM_QUANTILES_DEFAULT 10
#define SOLVER_TIMEOUT_DEFAULT 5

#define RECONSTRUCTION_METHOD_FORWARD	1
#define RECONSTRUCTION_METHOD_BACKWARD	2
#define RECONSTRUCTION_METHOD_SCALING	3

#define USE_FIXED_UNIFORM_CUTS_DEFAULT	0

#define ALPHA_D_DEFAULT 5

using namespace std;
using namespace cv;

class Algorithms {
public:
	Algorithms();
	Algorithms(LoadBalancingConfig config);
	void LoadNewConfig(LoadBalancingConfig config);
	void SetImageParameters(int width, int height, double overlap);
	void SetNumQuantiles(int Q);
	void SetTargetKeypoints(int target_num_keypoints);
	void AddKeypoints(vector<KeyPoint>& kpts);
	float GetNextDetectionThreshold();
	void CutVectorOptimization(int num_cooperators, vector<double>& c, vector<double>& p);
	std::vector<int> getCutVector();
	float getCompletionTime();
	void setInitialDetectionThreshold(double th);
	void reset();
	double getAlphad();
//		void UpdateNewImgParams(int h, int w, int M);
//	void UpdateNewIPscores(const std::vector<float>* IPscores);
//	void UpdateNodeInfo();

private:
	void UpdateBDR();
	void UpdateFDR();
	int lp_matrix_formulation(vector<double>& c, vector<double>& p);
	void lp_create_model();
	void set_uniform_quantiles();
//	void lp_update_quantiles();
//	void lp_update_nodes();
	int lp_solve_model();
//	int lp_find_opt_num_nodes();
	void lp_delete_model();
	int Mopt_;
	int lastM;
	float lastThreshold_;
	std::vector<float> lastIPscores_;
	std::vector<float> bdr_;
	float fdr_;

	std::vector<int> IPx_quantile_aprox_;
//	int num_available_nodes_;
	int num_using_nodes_;
	REAL *lp_D;
	REAL *lp_G;
	REAL *lp_E;
	REAL overlap_;
	int width_;
	int height_;
	lprec *lp;
	std::vector<int> optimal_cutvector_;
	float est_completion_time_;

	bool is_lpmodel_created_;
	bool quantiles_available_;
//	bool nodes_changed_;
//	bool quantiles_changed_;

	//Configs:
	int reconstruction_method_;
	float bdr_update_coef_;
	float fdr_update_coef_;
	float scaling_coef_;

	int num_quantiles_;
	float solver_timeout_; //seconds

	int use_fixed_uniform_cuts_;
	
	double alpha_d_;
};

#endif /* ALGORITHMS_H_ */
