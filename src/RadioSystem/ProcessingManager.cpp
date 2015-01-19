/*
 * ProcessingManager.cpp
 *
 *  Created on: 17/jan/2015
 *      Author: Alejandro Guillen
 */
#include <stdlib.h>
#include <iostream>
#include <vector>
#include "NodeManager/NodeManager.h"
#include "Tasks/Tasks.h"
#include "Messages/DataATCMsg.h"
#include "Messages/StartDATCMsg.h"
#include "Messages/CoopInfoMsg.h"
#include "Messages/ACKsliceMsg.h"
#include "Messages/AddCameraMsg.h"
#include "RadioSystem/OffloadingManager.h"
#include "RadioSystem/ProcessingManager.h"


using namespace std;

ProcessingManager::ProcessingManager(NodeManager* nm, int i){
	{
		//processcond.push_back(false);
		//processcond.reserve(2);

		//processcond[i] = 0;
		//thread_mutex.reserve(2);
		//thread_mutex.push_back(new boost::mutex);
		
		//boost::mutex m_mutex[i];
		//boost::condition m_condition[i];
		BRISK_detParams detPrms(60,4);
		BRISK_descParams dscPrms;
		extractor = new VisualFeatureExtraction();
		extractor->setDetector("BRISK", &detPrms);
		extractor->setDescriptor("BRISK",&dscPrms);
		encoder = new VisualFeatureEncoding();
		
		processcond = false;
		node_manager = nm;
		//cameraList.reserve(2);
		frame_id = -1;
		next_detection_threshold = 0;
		waitcamera=true;
		p_thread = boost::thread(&ProcessingManager::Processing_thread_cooperator, this, i);

	}
}

void ProcessingManager::start(int i){

		boost::mutex::scoped_lock lock(thread_mutex);
		processcond = true;
		//thread_condition[i].notify_one();
		thread_condition.notify_one();

}

void ProcessingManager::addCameraData(DATC_param_t* datc_param_camera, DataCTAMsg* msg, Connection* c){
	camera temp_cam;

	temp_cam.connection = c;
	temp_cam.id = msg->getSource();
	temp_cam.destination = msg->getDestination();
	//parameters
	temp_cam.detection_threshold = datc_param_camera->detection_threshold;
	temp_cam.max_features = datc_param_camera->max_features;
	//Data
	OCTET_STRING_t oct_data = msg->getData();
	uint8_t* imbuf = oct_data.buf;
	int data_size = oct_data.size;
	vector<uchar> jpeg_bitstream;
	for(int i=0;i<data_size;i++){
		jpeg_bitstream.push_back(imbuf[i]);
	}
	temp_cam.data = jpeg_bitstream;
	//Set initial values for the parameters:
	temp_cam.bandwidth = 20e6;
	temp_cam.Pdpx = 3.2e6;
	temp_cam.Pdip = 10000;
	temp_cam.Pe = 1000;
	//put on the list

	/*
	auto it = cameraList.begin();
	it=it+temp_cam.id;
	it--;
	cameraList.insert(it,temp_cam);
	*/
	//int i = temp_cam.id - 1;
	//cameraList[i] = temp_cam;
	cameraList = temp_cam;

}

void ProcessingManager::sendWiFiMessage(int i, Message *msg){
		
	msg->setTcpConnection(cameraList.connection);
	msg->setSource(cameraList.destination);
	msg->setDestination(i+1);
			
	node_manager->AddTask(msg);
}


void ProcessingManager::Processing_thread_cooperator(int i){
while(1){

	boost::mutex::scoped_lock lock(thread_mutex);
	while(processcond == false){
		thread_condition.wait(lock);
	}
	cout << "PM: I'm entering the Processing thread "<< i+1 << endl;
	
	
	cv::Mat slice;
	slice = imdecode(cameraList.data,CV_LOAD_IMAGE_GRAYSCALE);

	//send ACK_SLICE_MESSAGE
	cout << "Sending ACK_SLICE_MESSAGE to Camera " << i + 1 << endl;
	ACKsliceMsg *ackslice_msg = new ACKsliceMsg(frame_id);
	sendWiFiMessage(i, ackslice_msg);
	cout << "NM: exiting the wifi tx thread" << endl;
	
	// Extract the keypoints
	//BRISK_detParams detPrms(60,4);
	//BRISK_descParams dscPrms;
	
	//extractor = new VisualFeatureExtraction();
	
	//extractor->setDetector("BRISK", &detPrms);
	//extractor->setDescriptor("BRISK",&dscPrms);
		
	extractor->setDetThreshold("BRISK",cameraList.detection_threshold);
	double detTime = getTickCount();
	vector<KeyPoint> keypoints;
	extractor->extractKeypoints(slice,keypoints);
	detTime = (getTickCount()-detTime)/getTickFrequency();

	cout << "PM: ended extract_keypoints_task" << endl;
	cout << "extracted " << (int)keypoints.size() << "keypoints" << endl;
    cerr << "extracted " << (int)keypoints.size() << "keypoints\tDetThreshold=" << cameraList.detection_threshold << endl;

	//Extract features
	std::cout<<std::dec;
	
	double descTime = getTickCount();
	cv::Mat features;
	
	extractor->extractFeatures(slice,keypoints,features);
	descTime = (getTickCount()-descTime)/getTickFrequency();
	extractor->cutFeatures(keypoints,features,cameraList.max_features);
	
	cout << "now extracted " << (int)keypoints.size() << " keypoints" << endl;

	//features serialization
	vector<uchar> ft_bitstream;
	vector<uchar> kp_bitstream;
	//encoder = new VisualFeatureEncoding();
	int method = 0;
	/*if(method==0)// dummy
	{
		double fencTime = getTickCount();
		encoder->dummy_encodeBinaryDescriptors("BRISK",
				features,
				ft_bitstream);
		fencTime = (getTickCount()-fencTime)/getTickFrequency();
	}
	else // entropy coding
	{
		double fencTime = getTickCount();
		encoder->encodeBinaryDescriptors("BRISK",
				features,
				ft_bitstream);
		fencTime = (getTickCount()-fencTime)/getTickFrequency();
	}*/
	//cout << "PM: ended encode_features_task" << endl;
	
	double fencTime = getTickCount();
	encoder->dummy_encodeBinaryDescriptors("BRISK",
			features,
			ft_bitstream);
	fencTime = (getTickCount()-fencTime)/getTickFrequency();

	/*if(method==0){ // dummy
		double kencTime = getTickCount();
		encoder->dummy_encodeKeyPoints(keypoints,kp_bitstream);
		kencTime = (getTickCount()-kencTime)/getTickFrequency();

	}
	else{
		double kencTime = getTickCount();
		encoder->encodeKeyPoints(keypoints,kp_bitstream,640,480,true);
		kencTime = (getTickCount()-kencTime)/getTickFrequency();
	}*/
	double kencTime = getTickCount();
	encoder->dummy_encodeKeyPoints(keypoints,kp_bitstream);
	kencTime = (getTickCount()-kencTime)/getTickFrequency();

	cout << "sending " << (int)(keypoints.size()) << "keypoints" << endl;
	cout << "and " << (int)(features.rows) << "features" << endl;

	//send DataATCMsg
	cout << "send DataATCMsg to Camera" << i + 1 << endl;
	DataATCMsg *atc_msg = new DataATCMsg(frame_id, 0, 1, detTime, descTime, kencTime, fencTime, 0, features.rows, keypoints.size(), ft_bitstream, kp_bitstream);
	sendWiFiMessage(i, atc_msg);

	cout << "PM: exiting the wifi tx thread" << i+1 << endl;
	delete(atc_msg);
	//cur_state = IDLE;
	processcond = false;
	//removeCamera(cameraList[i].connection); not necessary 
}
}

void ProcessingManager::removeCamera(Connection* c){

	/*for(int i=0;i<cameraList.size();i++){
		camera temp_cam = cameraList[i];
		if(temp_cam.connection == c){
			std::cerr << "removed Data Camera " << temp_cam.id << endl;
			int m = temp_cam.id;

			cameraList.erase(cameraList.begin()+i);
		}
	}*/
}
