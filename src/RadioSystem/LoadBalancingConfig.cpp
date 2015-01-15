/*
 * LoadBalancingConfig.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: jordi
 */

#include "LoadBalancingConfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

LoadBalancingConfig::LoadBalancingConfig(int _reconstruction_method,
		float _bdr_update_coef, float _fdr_update_coef, float _scaling_coef,
		int _num_quantiles, float _solver_timeout, int _multicast_enabled) {
	reconstruction_method=_reconstruction_method;
	bdr_update_coef=_bdr_update_coef;
	fdr_update_coef=_fdr_update_coef;
	scaling_coef=_scaling_coef;

	num_quantiles=_num_quantiles;
	solver_timeout=_solver_timeout;
	multicast_enabled=_multicast_enabled;
	use_fixed_uniform_cuts=0;
}

LoadBalancingConfig::LoadBalancingConfig(int _reconstruction_method,
		float _bdr_update_coef, float _fdr_update_coef, float _scaling_coef,
		int _num_quantiles, float _solver_timeout, int _multicast_enabled,
		int _use_fixed_uniform_cuts) {
	reconstruction_method=_reconstruction_method;
	bdr_update_coef=_bdr_update_coef;
	fdr_update_coef=_fdr_update_coef;
	scaling_coef=_scaling_coef;

	num_quantiles=_num_quantiles;
	solver_timeout=_solver_timeout;
	multicast_enabled=_multicast_enabled;
	use_fixed_uniform_cuts=_use_fixed_uniform_cuts;
}

int LoadBalancingConfig::ParseConfigFile(std::string path){
	std::ifstream fid;
	fid.open(path.c_str(), std::fstream::in);

	if(fid==NULL){
		fprintf(stderr, "Could not open LoadBalancingConfig settings file\n");
		return -1;
	}
	while(!fid.eof()){
		std::string identifier = ReadIdentifier(&fid);
		std::string value = ReadUntilNewLine(&fid);
		if(identifier=="END"){
			break;
		}
		else if(strcmp(identifier.c_str(), "reconstruction_method") == 0){
			if(strcmp(value.c_str(), "forward") == 0){
				reconstruction_method = 1;
			}
			else if(strcmp(value.c_str(), "backward") == 0){
				reconstruction_method = 2;
			}
			else{ //Scaling
				reconstruction_method = 3;
			}
		}
		else if(strcmp(identifier.c_str(), "bdr_update_coef") == 0){
			bdr_update_coef = atof(value.c_str());
		}
		else if(strcmp(identifier.c_str(), "fdr_update_coef") == 0){
			fdr_update_coef = atof(value.c_str());
		}
		else if(strcmp(identifier.c_str(), "scaling_coef") == 0){
			scaling_coef = atof(value.c_str());
		}
		else if(strcmp(identifier.c_str(), "num_quantiles") == 0){
			num_quantiles = atoi(value.c_str());
		}
		else if(strcmp(identifier.c_str(), "solver_timeout") == 0){
			solver_timeout = atof(value.c_str());
		}
		else if(strcmp(identifier.c_str(), "multicast_enabled") == 0){
			if(strcmp(value.c_str(), "true") == 0){
				multicast_enabled = 1;
			}
			else{
				multicast_enabled = 0;
			}
		}
		else if(strcmp(identifier.c_str(), "use_fixed_uniform_cuts") == 0){
			if(strcmp(value.c_str(), "true") == 0){
				use_fixed_uniform_cuts = 1;
			}
			else{
				use_fixed_uniform_cuts = 0;
			}
		}
	}
	fid.close();
	return 0;
}

std::string LoadBalancingConfig::ReadIdentifier(std::ifstream* fid) {
	std::string line;
	std::getline(*fid, line, '=');
	return line;
}

LoadBalancingConfig::LoadBalancingConfig() {
}

std::string LoadBalancingConfig::ReadUntilNewLine(std::ifstream* fid) {
	std::string line;
	std::getline(*fid, line, '\n');
	return line;
}
