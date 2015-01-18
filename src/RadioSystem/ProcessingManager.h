/*
 * ProcessingManager.h
 *
 *  Created on: 17/jan/2015
 *      Author: Alejandro Guillen
 */

#ifndef PROCESSINGMANAGER_H_
#define PROCESSINGMANAGER_H_
#include <opencv2/opencv.hpp>
#include <vector>
#include "NodeManager/NodeManager.h"
#include "LoadBalancing.h"
#include "ProcessingSpeedEstimator.h"
#include "TxSpeedEstimator.h"
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;
using namespace cv;

#define COMPRESS_IMAGE 0
#define INITIAL_DETECTION_THRESHOLD 50


class Connection;

typedef struct camera{
	Connection* connection;

	double bandwidth;
	//double CPUspeed;

	double Pdpx;
	double Pdip;
	double Pe;
	//int Nkeypoints;
	//int Npixels;

	//double detTime;
	//double descTime;
	//double kencTime;
	//double fencTime;
	vector<uchar> data;
	double detection_threshold;
	int max_features;
	int destination;
	int id; //ALEXIS 09/01 COOP ID
	
}camera;

class ProcessingManager{
public:
	ProcessingManager(NodeManager* nm);
	
	void addCameraData(DATC_param_t* datc_param_camera, DataCTAMsg* msg, Connection* c);
	void sendACKMessage(int i);
	void send_DATA_ATC_Message(int i, int frame_id, double detTime, double descTime, double kencTime, double fencTime, int numFeat, int numKpts, vector<uchar>& features_data, vector<uchar>& keypoints_data);
	void Processing_thread_cooperator();
	void removeCamera(Connection* c);

			
private:

	//int cameras_to_use;
	//int received_cameras;
	int frame_id;
	vector<camera> cameraList;

	NodeManager* node_manager;
	//used to store keypoints and features from cameras
	vector<KeyPoint> keypoint_buffer;
	Mat features_buffer;

	//double camDetTime, camDescTime, camkEncTime, camfEncTime;

	//LoadBalancing loadbalancing;
	double next_detection_threshold;
	//double start_time;
	//int next_cam;
	
	VisualFeatureExtraction *extractor;
	VisualFeatureEncoding *encoder;
	VisualFeatureDecoding *decoder;
	
	
	boost::thread p_thread;
	//boost::asio::io_service iop;
	//deadline timer for receiving data from all cameras
	//boost::asio::deadline_timer t;
	//boost::asio::io_service::work work;

	boost::mutex mut;
	
	std::set<int> id; //ALEXIS 09/01 COOP ID

};


#endif /* PROCESSINGMANAGER_H_ */
