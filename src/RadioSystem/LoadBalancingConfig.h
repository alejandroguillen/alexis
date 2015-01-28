/*
 * LoadBalancingConfig.h
 *
 *  Created on: Oct 8, 2014
 *      Author: jordi
 */

#ifndef LOADBALANCINGCONFIG_H_
#define LOADBALANCINGCONFIG_H_

#include <string>

class LoadBalancingConfig {
public:
	LoadBalancingConfig();
	LoadBalancingConfig(int _reconstruction_method, float _bdr_update_coef, float _fdr_update_coef, float _scaling_coef, int _num_quantiles, float _solver_timeout, double _alpha_d);
	LoadBalancingConfig(int _reconstruction_method, float _bdr_update_coef, float _fdr_update_coef, float _scaling_coef, int _num_quantiles, float _solver_timeout, int _use_fixed_uniform_cuts, double _alpha_d);
	int ParseConfigFile(std::string path);

	int reconstruction_method;
	float bdr_update_coef;
	float fdr_update_coef;
	float scaling_coef;

	int num_quantiles;
	float solver_timeout;
	int use_fixed_uniform_cuts;
	
	double alpha_d;

private:
	std::string ReadIdentifier(std::ifstream* fid);
	std::string ReadUntilNewLine(std::ifstream* fid);
};

#endif /* LOADBALANCINGCONFIG_H_ */
