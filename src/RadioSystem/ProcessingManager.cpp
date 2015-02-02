/*
 * ProcessingManager.cpp
 *
 *  Created on: 17/jan/2015
 *      Author: Alejandro Guillen
 */
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <highgui.h>
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
using namespace cv;

ProcessingManager::ProcessingManager(NodeManager* nm, int i){
	{
		//processcond.push_back(false);
		//processcond.reserve(2);

		//processcond[i] = 0;
		//thread_mutex.reserve(2);
		//thread_mutex.push_back(new boost::mutex);
		
		//boost::mutex m_mutex[i];
		//boost::condition m_condition[i];
		/*BRISK_detParams detPrms(60,4);
		BRISK_descParams dscPrms;
		extractor = new VisualFeatureExtraction();
		extractor->setDetector("BRISK", &detPrms);
		extractor->setDescriptor("BRISK",&dscPrms);
		encoder = new VisualFeatureEncoding();
		*/
		processcond = false;
		processempty=true;
		node_manager = nm;
		//cameraList.reserve(2);
		frame_id = -1;
		next_detection_threshold = 0;
		waitcamera=true;
		
		dataavailable=0;
		last_subslice_received=false;
		
		secondprocesscond=false;
		secondprocess=false;
		
		p_thread = boost::thread(&ProcessingManager::Processing_thread_cooperator, this, i);

	}
}


void ProcessingManager::start(){

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
	temp_cam.slices_total = datc_param_camera->num_cooperators;
	temp_cam.sub_slices_total = msg->getSliceNumber();
	temp_cam.slice_id = msg->getFrameId()+1;
	temp_cam.detection_threshold = datc_param_camera->detection_threshold;
	temp_cam.max_features = datc_param_camera->max_features;
	temp_cam.detTime = 0;
	temp_cam.descTime = 0;
	temp_cam.kptsSize = 0;

	count_subslices = 0;
	
	cameraList = temp_cam;
	
}


void ProcessingManager::addSubSliceData(DataCTAMsg* msg){
	subslice temp_subslice;

	count_subslices++;
	
	//parameters
	temp_subslice.sub_slice_id = count_subslices;
	//temp_subslice.slice_id = msg->getFrameId()+1;
	//temp_subslice.sub_slices_total = msg->getSliceNumber();
	temp_subslice.sub_slice_topleft = msg->getTopLeft();
	//temp_subslice.detection_threshold = datc_param_camera->detection_threshold;
	//temp_subslice.max_features = datc_param_camera->max_features;
	//Data
	OCTET_STRING_t oct_data = msg->getData();
	uint8_t* imbuf = oct_data.buf;
	int data_size = oct_data.size;
	vector<uchar> jpeg_bitstream;
	for(int i=0;i<data_size;i++){
		jpeg_bitstream.push_back(imbuf[i]);
	}
	temp_subslice.data = jpeg_bitstream;
	
	temp_subslice.last_subslice_received = false;
	if(cameraList.sub_slices_total==temp_subslice.sub_slice_id){
		temp_subslice.last_subslice_received = true;
	}
	//Set initial values for the parameters:
	
	//put on the list

	/*
	auto it = cameraList.begin();
	it=it+temp_subslice.id;
	it--;
	cameraList.insert(it,temp_subslice);
	*/
	//int i = temp_subslice.id - 1;
	//subsliceList[i] = temp_subslice;
	subsliceList.push_back(temp_subslice);
	
	//int j = subsliceList.size()-1;
	//subsliceList[j].cond == true;
	
	notifyToProcess(count_subslices);
	cout << "PM: added subslice " << endl;
}


void ProcessingManager::notifyToProcess(int i){
	
	//first slice need 2 subslices to start processing
	if(cameraList.slice_id == 1){
		if(i==2){ 
			start();
		}else if(i>2){
			setData();
		}		
	}
	//last slice needs 3 or 2 subslices to start processing
	else if(cameraList.slices_total == cameraList.slice_id){ //3 subslices
		if(i==3){ //3 subslices or more
			start();
		}else if(i>3){  //remaining subslices
			setData();
		}else if(i == cameraList.sub_slices_total){ //2subslices in total only
			start();
		}
		
		if(subsliceList[i-1].last_subslice_received == true){ //last subslice needs two subslices only
			setData();
		}
	}
	//other slices need 3 subslices to start processing
	else{ 
		if(i==3){ 
			start();
		}else if(i>3){  //remaining subslices
			setData();
		}
	}
	//send ACK to camera when Receiving all subslices
	if(subsliceList[i-1].last_subslice_received == true){
		node_manager->TransmissionFinished(cameraList.id, cameraList.connection);
	}
}


Mat ProcessingManager::mergeSubSlices(int subslices_iteration){
	
	slices temp_slice;
	int col_offset;
	temp_slice.last_subslices_iteration = false;
	
	if(cameraList.slice_id == 1 && cameraList.slices_total!=1){ //first slice
		if(subslices_iteration == 1){ //merge first and second subslices
		
			Mat subslice0 = imdecode(subsliceList[0].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice1 = imdecode(subsliceList[1].data, CV_LOAD_IMAGE_GRAYSCALE);
			Size sz0 = subslice0.size();
			Size sz1 = subslice1.size();	
			Mat slicedone(sz0.height, sz0.width+sz1.width, CV_LOAD_IMAGE_GRAYSCALE);	
			Mat left(slicedone, Rect(0, 0, sz0.width, sz0.height));	
			subslice0.copyTo(left);	
			Mat right(slicedone, Rect(sz0.width, 0, sz1.width, sz1.height));	
			subslice1.copyTo(right);	
			col_offset = subsliceList[0].sub_slice_topleft.xCoordinate;
			//imshow("slice", slicedone);
			temp_slice.id = subslices_iteration;
			temp_slice.col_offset = col_offset;
			
			sliceList.push_back(temp_slice);
			return slicedone;

		}
		else if(subslices_iteration<=cameraList.sub_slices_total){ //middle subslices, need three subslices
		
			int j=subslices_iteration;
			Mat subslice0 = imdecode(subsliceList[j-2].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice1 = imdecode(subsliceList[j-1].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice2 = imdecode(subsliceList[j].data, CV_LOAD_IMAGE_GRAYSCALE);
			Size sz0 = subslice0.size();
			Size sz1 = subslice1.size();	
			Size sz2 = subslice2.size();	
			Mat slice_op(sz0.height, sz0.width+sz1.width, CV_LOAD_IMAGE_GRAYSCALE);	
			Mat left_op(slice_op, Rect(0, 0, sz0.width, sz0.height));	
			subslice0.copyTo(left_op);	
			Mat right_op(slice_op, Rect(sz0.width, 0, sz1.width, sz1.height));	
			subslice1.copyTo(right_op);
			
			Size sslice_op = slice_op.size();
			Mat slicedone(sslice_op.height, sslice_op.width+sz2.width, CV_LOAD_IMAGE_GRAYSCALE);
			Mat left(slicedone, Rect(0, 0, sslice_op.width, sslice_op.height));
			subslice0.copyTo(left);	
			Mat right(slicedone, Rect(sz2.width, 0, sz2.width, sz2.height));	
			subslice1.copyTo(right);
			
			col_offset = subsliceList[j-2].sub_slice_topleft.xCoordinate;
			temp_slice.id = subslices_iteration;
			temp_slice.col_offset = col_offset;

			if(subslices_iteration == cameraList.sub_slices_total-1 && cameraList.slice_id != cameraList.slices_total){
				temp_slice.last_subslices_iteration = true;
			}

			sliceList.push_back(temp_slice);
			
			
			return slicedone;
		}
	}
	if(cameraList.slice_id != 1 || cameraList.slices_total==1){ //not first slice or only one slice in total (1 coop)
		
		if(cameraList.slice_id == cameraList.slices_total && subslices_iteration == cameraList.sub_slices_total-1){	// last slice and subslice, merge second to last and last subslices

			int j=subslices_iteration;
			Mat subslice0 = imdecode(subsliceList[j-1].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice1 = imdecode(subsliceList[j].data, CV_LOAD_IMAGE_GRAYSCALE);
			Size sz0 = subslice0.size();
			Size sz1 = subslice1.size();
			Mat slicedone(sz0.height, sz0.width+sz1.width, CV_LOAD_IMAGE_GRAYSCALE);
			Mat left_op(slicedone, Rect(0, 0, sz0.width, sz0.height));
			subslice0.copyTo(left_op);
			Mat right_op(slicedone, Rect(sz0.width, 0, sz1.width, sz1.height));
			subslice1.copyTo(right_op);

			col_offset = subsliceList[j-1].sub_slice_topleft.xCoordinate;

			temp_slice.last_subslices_iteration = true;
			temp_slice.id = subslices_iteration;
			temp_slice.col_offset = col_offset;

			sliceList.push_back(temp_slice);
			return slicedone;
		}
		
		else if(subslices_iteration < cameraList.sub_slices_total){ //need three subslices to merge

			int j=subslices_iteration+1;
			Mat subslice0 = imdecode(subsliceList[j-2].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice1 = imdecode(subsliceList[j-1].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice2 = imdecode(subsliceList[j].data, CV_LOAD_IMAGE_GRAYSCALE);
			Size sz0 = subslice0.size();
			Size sz1 = subslice1.size();	
			Size sz2 = subslice2.size();	
			Mat slice_op(sz0.height, sz0.width+sz1.width, CV_LOAD_IMAGE_GRAYSCALE);	
			Mat left_op(slice_op, Rect(0, 0, sz0.width, sz0.height));	
			subslice0.copyTo(left_op);	
			Mat right_op(slice_op, Rect(sz0.width, 0, sz1.width, sz1.height));	
			subslice1.copyTo(right_op);
			
			Size sslice_op = slice_op.size();
			Mat slicedone(sslice_op.height, sslice_op.width+sz2.width, CV_LOAD_IMAGE_GRAYSCALE);
			Mat left(slicedone, Rect(0, 0, sslice_op.width, sslice_op.height));
			subslice0.copyTo(left);	
			Mat right(slicedone, Rect(sz2.width, 0, sz2.width, sz2.height));	
			subslice1.copyTo(right);
			
			col_offset = subsliceList[j-2].sub_slice_topleft.xCoordinate;

			if(subslices_iteration == cameraList.sub_slices_total-2 && cameraList.slice_id != cameraList.slices_total){
				temp_slice.last_subslices_iteration = true;
			}
			temp_slice.id = subslices_iteration;
			temp_slice.col_offset = col_offset;

			sliceList.push_back(temp_slice);
			return slicedone;
		}
		
	}
	
}

void ProcessingManager::Processing_thread_cooperator(int i){
while(1){

	boost::mutex::scoped_lock lock(thread_mutex);
	while(processcond == false){
		thread_condition.wait(lock);
	}
	cout << "PM: I'm entering the Processing thread "<< i+1 << endl;


	subslices_iteration++;
	if(subslices_iteration!=1){
		getData();
	}
	
	//merge subslices to a slice
	Mat slice_merged = mergeSubSlices(subslices_iteration);
	
	//start processing timer
	if(subslices_iteration==1){
		processingTime = getTickCount();
	}
	
	//process resulting slice and save keypoints/features
	node_manager->Processing_slice(i,subslices_iteration, slice_merged,cameraList.detection_threshold, cameraList.max_features);
}
}



void ProcessingManager::storeKeypointsAndFeatures(int subslices_iteration,vector<KeyPoint>& kpts,Mat& features,
		double detTime, double descTime){

	//save features
	features_buffer.push_back(features);
	
	//sum parameters
	cameraList.detTime += detTime;
	cameraList.descTime += descTime;
	cameraList.kptsSize += kpts.size();
	
	//save keypoints
	int i = subslices_iteration-1;	
	for(int j=0;j<kpts.size();j++){
		kpts[j].pt.x = kpts[j].pt.x + sliceList[i].col_offset;
		keypoint_buffer.push_back(kpts[j]);
	}
		
	//if last slice in the Coop, do an average of the parameters and send to camera
	if(sliceList[i].last_subslices_iteration==true){
		
		//end processing timer
		processingTime = (getTickCount()-processingTime)/getTickFrequency();
			
		//average estimate parameters
		//averageParameters();
		
		//clear sliceList and subsliceList
		sliceList.clear();
		subsliceList.clear();
		//delete cameraList;

		last_subslice_received=false;
		dataavailable=0;

		//block thread until another first slice enters
		processcond = false;

		//encode kpts/features and send DataATCmsg to Camera
		node_manager->notifyCooperatorCompleted(cameraList.id,keypoint_buffer,features_buffer,cameraList.detTime,cameraList.descTime, processingTime, cameraList.connection);

		features_buffer.release();
		keypoint_buffer.clear();

		cout << "PM: Coop finished" << endl;
		
		secondprocess = true;
		secondprocesscond = false;
	}

}

void ProcessingManager::setData(){

	mutex.lock();
	dataavailable++;
	mutex.unlock();
}

void ProcessingManager::getData(){
	
	mutex.lock();
	while(dataavailable==0){
		mutex.unlock();
		mutex.lock();
	}
	dataavailable--;
	mutex.unlock();
}

/*
void ProcessingManager::averageParameters(){
	
	for(int j=0;j<sliceList.size();j++){
		
		kptsSize += sliceList[j].kpts_size;
		featuresSize += sliceList[j].features_size;
		detTime += sliceList[j].detTime
		descTime += sliceList[j].descTime 
	}
}
*/
