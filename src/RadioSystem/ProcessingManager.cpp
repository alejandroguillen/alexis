/*
 * ProcessingManager.cpp
 *
 *  Created on: 17/jan/2015
 *      Author: Alejandro Guillen
 */
#include <stdlib.h>
#include <iostream>
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
/*void ProcessingManager::set_radioSystem(RadioSystem *rs){
	radioSystem_ptr = rs;
}*/

ProcessingManager::ProcessingManager(NodeManager* nm){
	//io(),
	//t(io),
	//work(io)
	{
		cameras_to_use = 0;
		received_cameras = 0;
		node_manager = nm;
		frame_id = -1;
		next_detection_threshold = 0;
		//startTimer();
	}
}	

void ProcessingManager::addCameraData(vector<DATC_param_t> datc_param_camera, DataCTAMsg* msg, Connection* c){
	camera temp_cam;


	//Connection
	/*std::set<Connection*> connections = radioSystem_ptr->getWiFiConnections();
	std::set<Connection*>::iterator c = connections.begin();
	int tmp = connections.size();
	if(tmp > 1){   //if because Camera1 off -> error Camera2
		int ack = msg->getSource();
		ack--;
		std::advance(it, ack);
	}*/
	temp_cam.connection = c;
	
	temp_cam.id = msg->getSource();
	temp_cam.destination = msg->getDestination();

	//parameters

	temp_cam.detection_threshold = datc_param_camera[temp_cam.id].detection_threshold;
	temp_cam.max_features = datc_param_camera[temp_cam.id].max_features;

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
	
	cameraList.push_back(temp_cam);
}

void ProcessingManager::sendACKMessage(int i){
	
	ACKsliceMsg *ackslice_msg = new ACKsliceMsg(frame_id);
	ackslice_msg->setTcpConnection(cameraList[i].connection);
	ackslice_msg->setSource(cameraList[i].destination);
	ackslice_msg->setDestination(i);
	cameraList[i].connection->writeMsg(ackslice_msg);
			
	delete(ackslice_msg);		
}

void ProcessingManager::send_DATA_ATC_Message(int i, int frame_id, double detTime, double descTime, double kencTime, double fencTime, int numFeat, int numKpts, vector<uchar>& features_data, vector<uchar>& keypoints_data){

	DataATCMsg *atc_msg = new DataATCMsg(frame_id, 0, 1, detTime, descTime, kencTime, fencTime, 0, numFeat, numKpts, features_data, keypoints_data);
	atc_msg->setTcpConnection(cameraList[i].connection);
	atc_msg->setSource(cameraList[i].destination);
	atc_msg->setDestination(i);
	cameraList[i].connection->writeMsg(atc_msg);
			
	delete(atc_msg);	
}

void ProcessingManager::Processing_thread_cooperator(int i){
	cout << "NM: I'm entering the Processing thread " << endl;


	//boost::mutex monitor;
	//boost::mutex::scoped_lock lk(monitor);

	//decode the image (should become a task)
	cv::Mat slice;
	slice = imdecode(cameraList[i].data,CV_LOAD_IMAGE_GRAYSCALE);

	
	//send ACK_SLICE_MESSAGE
	cout << "Sending ACK_SLICE_MESSAGE to Camera " << i << endl;
	//LOCK
	sendACKMessage(i);
	//UNLOCK
	cout << "NM: exiting the wifi tx thread" << endl;
	
	
	// Extract the keypoints
	//cur_task = new ExtractKeypointsTask(extractor,slice,datc_param_camera[i].detection_threshold);
	
	BRISK_detParams detPrms(60,4);
	BRISK_descParams dscPrms;
	
	extractor = new VisualFeatureExtraction();
	
	extractor->setDetector("BRISK", &detPrms);
	extractor->setDescriptor("BRISK",&dscPrms);
		
	extractor->setDetThreshold("BRISK",cameraList[i].detection_threshold);
	double detTime = getTickCount();
	vector<KeyPoint> keypoints;
	extractor->extractKeypoints(slice,keypoints);
	detTime = (getTickCount()-detTime)/getTickFrequency();

	cout << "NM: ended extract_keypoints_task" << endl;

	cout << "extracted " << (int)keypoints.size() << "keypoints" << endl;
    cerr << "extracted " << (int)keypoints.size() << "keypoints\tDetThreshold=" << cameraList[i].detection_threshold << endl;


	//Extract features
	std::cout<<std::dec;
	//cur_task = new ExtractFeaturesTask(extractor,slice,keypoints,datc_param_camera[i].max_features);
	
	double descTime = getTickCount();
	
	cv::Mat features;
	
	extractor->extractFeatures(slice,keypoints,features);
	descTime = (getTickCount()-descTime)/getTickFrequency();
	extractor->cutFeatures(keypoints,features,cameraList[i].max_features);
	
	cout << "now extracted " << (int)keypoints.size() << " keypoints" << endl;

	
	//features serialization
	vector<uchar> ft_bitstream;
	vector<uchar> kp_bitstream;
	//cur_task = new EncodeFeaturesTask(encoder,"BRISK",features,0);
	
	encoder = new VisualFeatureEncoding();
	
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

	//cout << "NM: ended encode_features_task" << endl;
	double fencTime = getTickCount();
	encoder->dummy_encodeBinaryDescriptors("BRISK",
			features,
			ft_bitstream);
	fencTime = (getTickCount()-fencTime)/getTickFrequency();

	//cur_task = new EncodeKeypointsTask(encoder,keypoints,640,480,true,0);
	
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
	cout << "send DataATCMsg to Camera" << i << endl;
	//LOCK
	send_DATA_ATC_Message(i, frame_id, detTime, descTime, kencTime, fencTime, features.rows, keypoints.size(), ft_bitstream, kp_bitstream);
	//UNLOCK
	cout << "NM: exiting the wifi tx thread" << endl;

	//cur_state = IDLE;
	removeCamera(cameraList[i].connection);
}

void ProcessingManager::removeCamera(Connection* c){
	for(int i=0;i<cameraList.size();i++){
		camera temp_cam = cameraList[i];
		if(temp_cam.connection == c){
			//delete temp_cam.processing_speed_estimator;
			//delete temp_cam.tx_speed_estimator;
			//ALEXIS 09/01 COOP ID
			std::cerr << "removed Data Camera " << temp_cam.id << endl;
			int m = temp_cam.id;
			id.erase(m);
			//
			cameraList.erase(cameraList.begin()+i);
		}
	}
}
