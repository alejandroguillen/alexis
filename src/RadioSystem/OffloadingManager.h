/*
 * OffloadingManager.h
 *
 *  Created on: 19/set/2014
 *      Author: greeneyes
 */

#ifndef OFFLOADINGMANAGER_H_
#define OFFLOADINGMANAGER_H_
#include <opencv2/opencv.hpp>
#include <vector>
#include "NodeManager/NodeManager.h"
#include "LoadBalancing.h"
#include "ProcessingSpeedEstimator.h"
#include "TxSpeedEstimator.h"
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "Coef.h"

using namespace std;
using namespace cv;

#define COMPRESS_IMAGE 0
#define INITIAL_DETECTION_THRESHOLD 50


class Connection;

typedef struct cooperator{
	Connection* connection;

	double bandwidth;
	double CPUspeed;

	double Pdpx;
	double Pdip;
	double Pe;
	int Nkeypoints;
	int Npixels;
	ProcessingSpeedEstimator* processing_speed_estimator;
	TxSpeedEstimator* tx_speed_estimator;
	double idleTime;
	double txTime;
	double completionTime;

	double load;
	Mat image_slice;
	int col_offset;

	double detTime;
	double descTime;
	double kencTime;
	double fencTime;
	
	int id; //ALEXIS 09/01 COOP ID
	Coef* processing_time_coef;
	double Ptcoef;
	double processingTime;
	
}cooperator;

class OffloadingManager{
public:
	OffloadingManager(NodeManager* nm):
	io(),
	t(io),
	work(io)
	{
		cooperators_to_use = 0;
		received_cooperators = 0;
		node_manager = nm;
		next_detection_threshold = 0;
		startTimer();
	}

	//add keypoints and features from cooperators
	void addKeypointsAndFeatures(vector<KeyPoint>& kpts,Mat& features,Connection* cn,
			double detTime, double descTime, double kencTime, double processingTime);

	//reset variables and keep track of progresses
//	void createOffloadingTask(int num_cooperators);
	void createOffloadingTask(int num_cooperators, int target_num_keypoints);

	void addCooperator(Connection* c);
	void removeCooperator(Connection* c);
	Mat computeLoads(Mat& image);
	double getNextDetectionThreshold();
	void estimate_parameters(cooperator* coop);
	void transmitLoads();
	void transmitStartDATC(StartDATCMsg* msg);
	int probeLinks();
	void sortCooperators();

	void timerExpired(const boost::system::error_code& error);
	void startTimer();
	void runThread();
	int getNumAvailableCoop();
	void transmitNextCoop();
	void notifyACKslice(int frameID, Connection* cn);
	//void printKeypoints(vector<KeyPoint>& kpts); //Debug

private:

	int cooperators_to_use;
	int received_cooperators;
	vector<cooperator> cooperatorList;

	NodeManager* node_manager;
	//used to store keypoints and features from cooperators
	vector<KeyPoint> keypoint_buffer;
	Mat features_buffer;

	double camDetTime, camDescTime, camkEncTime, camfEncTime;

	LoadBalancing loadbalancing;
	double next_detection_threshold;
	double start_time;
	int next_coop;

	boost::thread r_thread;
	boost::asio::io_service io;
	//deadline timer for receiving data from all cooperators
	boost::asio::deadline_timer t;
	boost::asio::io_service::work work;

	boost::mutex mut;
	
	std::set<int> id; //ALEXIS 09/01 COOP ID

};


#endif /* OFFLOADINGMANAGER_H_ */
