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
#include <boost/thread.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

using namespace std;
using namespace cv;

#define COMPRESS_IMAGE 0
#define INITIAL_DETECTION_THRESHOLD 50

class Connection;

typedef struct camera{
	Connection* connection;
	double bandwidth;
	double Pdpx;
	double Pdip;
	double Pe;
	vector<uchar> data;
	double detection_threshold;
	int max_features;
	int destination;
	int id; 
	
}camera;

class ProcessingManager{
public:
	ProcessingManager(NodeManager* nm, int i);
	
	void start(int i);
	void addCameraData(DATC_param_t* datc_param_camera, DataCTAMsg* msg, Connection* c);
	void sendWiFiMessage(int i, Message *msg);
	void Processing_thread_cooperator(int i);
	void removeCamera(Connection* c);
			
private:

	int frame_id;
	double next_detection_threshold;
	//vector<camera> cameraList;
	camera cameraList;

	NodeManager* node_manager;
	
	VisualFeatureExtraction *extractor;
	VisualFeatureEncoding *encoder;
	VisualFeatureDecoding *decoder;
	
	boost::thread p_thread;
	
	bool waitcamera;
	boost::mutex m_mutex;
	//boost::condition m_condition;
	
	//vector<bool> processcond;
	bool processcond;
	//boost::ptr_vector<boost::mutex> thread_mutex;
	boost::mutex thread_mutex;
	//boost::ptr_vector<boost::condition> thread_condition;
	boost::condition thread_condition;
		
};

#endif /* PROCESSINGMANAGER_H_ */
