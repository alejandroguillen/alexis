/*
 * SIMULATIONManager.h
 *
 *  Created on: 17/jan/2015
 *      Author: Alejandro Guillen
 */

#ifndef SIMULATIONMANAGER_H_
#define SIMULATIONMANAGER_H_
#include <opencv2/opencv.hpp>
#include <vector>
#include "NodeManager/NodeManager.h"
#include "RadioSystem/RadioSystem.h"
#include "RadioSystem/Algorithms.h"
#include "RadioSystem/ProcessingSpeedEstimator.h"
#include "RadioSystem/TxSpeedEstimator.h"
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

using namespace std;
using namespace cv;

class Connection;


class SIMULATIONManager{
public:
	SIMULATIONManager(RadioSystem* rs,NodeManager* nm);
	void start();
	void SIMULATION_thread();
	

private:
	
	NodeManager* node_manager;
	RadioSystem* radio_system;
	int count_subslices;

	int frame_id;

	double processingTime;
	
	boost::thread p_thread;
	
	
	//vector<bool> processcond;
	bool simulationcond;

	//boost::ptr_vector<boost::mutex> thread_mutex;
	boost::mutex simulation_mutex;

	//boost::ptr_vector<boost::condition> thread_condition;
	boost::condition simulation_condition;
	
	
	boost::mutex mutex;
	
		
};

#endif /* SIMULATIONMANAGER_H_ */
