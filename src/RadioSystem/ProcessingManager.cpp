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
#include "opencv2/highgui/highgui.hpp"
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
		processingTime =0;
		mergeTime = 0;
		processcond = false;
		node_manager = nm;
		frame_id = -1;
		next_detection_threshold = 0;

		dataavailable=0;
		last_subslice_received=false;

		subslices_iteration = 0;
		count_subslices=0;

		p_thread = boost::thread(&ProcessingManager::Processing_thread_cooperator, this, i);

	}
}


void ProcessingManager::start(){

		boost::mutex::scoped_lock lock(thread_mutex);
		processcond = true;
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
	temp_subslice.sub_slice_topleft = msg->getTopLeft();
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
	subsliceList.push_back(temp_subslice);

	cerr << "PM: added subslice from Camera " << cameraList.id << " of "<< cameraList.sub_slices_total <<" total subslices"<< endl;
	setData();

	//send ACK to camera when Receiving all subslices
	if(subsliceList[count_subslices-1].last_subslice_received == true){
		node_manager->TransmissionFinished(cameraList.id, cameraList.connection);
	}
}


Mat ProcessingManager::mergeSubSlices(int subslices_iteration, vector<subslice> subsliceList){

	slices temp_slice;
	int col_offset;
	temp_slice.last_subslices_iteration = false;

	mergeTime = getTickCount();
	Mat slicedone;

	//ONE COOP only
	if(cameraList.slice_id == 1 && cameraList.slice_id == cameraList.slices_total)
	{
		//first slice needs 2 subslicess
		if(subslices_iteration == 1)
		{
			Mat subslice0 = imdecode(subsliceList[0].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice1 = imdecode(subsliceList[1].data, CV_LOAD_IMAGE_GRAYSCALE);
			hconcat(subslice0,subslice1,slicedone);
			col_offset = subsliceList[0].sub_slice_topleft.xCoordinate;

			temp_slice.id = subslices_iteration;
			temp_slice.col_offset = col_offset;
			temp_slice.mergeTime = (getTickCount()-mergeTime)/getTickFrequency();
			sliceList.push_back(temp_slice);

			return slicedone;
		}
		//middle slices need 3 subslices
		else if(subslices_iteration > 1 && subslices_iteration !=cameraList.sub_slices_total)
		{
			int j=subslices_iteration;
			Mat subslice0 = imdecode(subsliceList[j-2].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice1 = imdecode(subsliceList[j-1].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice2 = imdecode(subsliceList[j].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat slice_op;
			hconcat(subslice0,subslice1,slice_op);
			hconcat(slice_op,subslice2,slicedone);
			col_offset = subsliceList[j-2].sub_slice_topleft.xCoordinate;

			temp_slice.id = subslices_iteration;
			temp_slice.col_offset = col_offset;
			temp_slice.mergeTime = (getTickCount()-mergeTime)/getTickFrequency();
			sliceList.push_back(temp_slice);

			return slicedone;
		}
		//last slice needs 2 subslices
		else if(subslices_iteration == cameraList.sub_slices_total)
		{
			int j=subslices_iteration;
			Mat subslice0 = imdecode(subsliceList[j-2].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice1 = imdecode(subsliceList[j-1].data, CV_LOAD_IMAGE_GRAYSCALE);
			hconcat(subslice0,subslice1,slicedone);
			col_offset = subsliceList[j-2].sub_slice_topleft.xCoordinate;

			temp_slice.id = subslices_iteration;
			temp_slice.col_offset = col_offset;
			temp_slice.last_subslices_iteration = true;
			temp_slice.mergeTime = (getTickCount()-mergeTime)/getTickFrequency();
			sliceList.push_back(temp_slice);

			return slicedone;
		}
	}
	//MORE than 1 COOP
	else
	{
		//first coop needs 2 subslices for first slice
		if(cameraList.slice_id == 1 && subslices_iteration == 1)
		{
			Mat subslice0 = imdecode(subsliceList[0].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice1 = imdecode(subsliceList[1].data, CV_LOAD_IMAGE_GRAYSCALE);
			hconcat(subslice0,subslice1,slicedone);
			temp_slice.id = subslices_iteration;
			temp_slice.col_offset = col_offset;
			if(subslices_iteration == cameraList.sub_slices_total-1){
				temp_slice.last_subslices_iteration = true;
			}
			temp_slice.mergeTime = (getTickCount()-mergeTime)/getTickFrequency();
			sliceList.push_back(temp_slice);

			return slicedone;
		}
		//others coops (not last) needs 3 subslices
		else if(subslices_iteration >= 1 && cameraList.slice_id != cameraList.slices_total)
		{
			int j;
			if(cameraList.slice_id == 1) j=subslices_iteration;
			else j=subslices_iteration+1;

			Mat subslice0 = imdecode(subsliceList[j-2].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice1 = imdecode(subsliceList[j-1].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat subslice2 = imdecode(subsliceList[j].data, CV_LOAD_IMAGE_GRAYSCALE);
			Mat slice_op;

			if(j == cameraList.sub_slices_total-1){
				temp_slice.last_subslices_iteration = true;
				//cut last subslice in order to get good concatenation with penultimate
				hconcat(subslice0,subslice1,slice_op);
				Size sz0 = slice_op.size();
				Size sz2 = subslice1.size();
				Size sz1 = subslice2.size();


				//know where is the last column with the same information (image) of the penultimate subslice
				int xcoordinateLast = (subsliceList[j-1].sub_slice_topleft.xCoordinate+sz2.width-subsliceList[j].sub_slice_topleft.xCoordinate);

				cerr << "xcoordinateLast: "<< xcoordinateLast << " width penultimate: "<<sz2.width<< " width last: " <<sz1.width<< " topleft penultimate: " <<subsliceList[j-1].sub_slice_topleft.xCoordinate<<" topleft last: " << subsliceList[j].sub_slice_topleft.xCoordinate<< endl;

				if(xcoordinateLast!=sz1.width && xcoordinateLast >0){
					//cut to get only the different information of the image (remove the repeated image part)
					Mat subslicecut;
					int s1 = xcoordinateLast;
					int s2 = sz1.width;

					subslicecut = Mat(subslice2, Range(0,sz1.height), Range(s1,s2)).clone();

					//fill with 0s the remaining size of the subslice (minimum size is the overlap)
					Mat blackImage = Mat::zeros(Size(sz1.width-subslicecut.cols,sz1.height),subslicecut.type());

					//Mat sliceLarge;
					hconcat(subslicecut,blackImage,subslice2);
				}

				//merge with the correct subslice image
				hconcat(slice_op,subslice2,slicedone);
				col_offset = subsliceList[j-2].sub_slice_topleft.xCoordinate;

				temp_slice.id = subslices_iteration;
				temp_slice.col_offset = col_offset;

				temp_slice.mergeTime = (getTickCount()-mergeTime)/getTickFrequency();
				sliceList.push_back(temp_slice);

				return slicedone;
			}

			hconcat(subslice0,subslice1,slice_op);
			hconcat(slice_op,subslice2,slicedone);
			col_offset = subsliceList[j-2].sub_slice_topleft.xCoordinate;

			temp_slice.id = subslices_iteration;
			temp_slice.col_offset = col_offset;

			temp_slice.mergeTime = (getTickCount()-mergeTime)/getTickFrequency();
			sliceList.push_back(temp_slice);

			return slicedone;
		}
		//last coop
		else if(cameraList.slice_id == cameraList.slices_total)
		{
			//first and middle slices need 3 subslices
			if(subslices_iteration >=1 &&  subslices_iteration != cameraList.sub_slices_total-1)
			{
				int j=subslices_iteration+1;
				Mat subslice0 = imdecode(subsliceList[j-2].data, CV_LOAD_IMAGE_GRAYSCALE);
				Mat subslice1 = imdecode(subsliceList[j-1].data, CV_LOAD_IMAGE_GRAYSCALE);
				Mat subslice2 = imdecode(subsliceList[j].data, CV_LOAD_IMAGE_GRAYSCALE);
				Mat slice_op;

				hconcat(subslice0,subslice1,slice_op);

				if(j == cameraList.sub_slices_total-1){
				//cut last subslice in order to get good concatenation with penultimate

					Size sz0 = subslice1.size();
					Size sz1 = subslice2.size();

					//know where is the last column with the same information (image) of the penultimate subslice
					int xcoordinateLast = (subsliceList[j-1].sub_slice_topleft.xCoordinate+sz0.width-subsliceList[j].sub_slice_topleft.xCoordinate);
					cerr << "xcoordinateLast: "<< xcoordinateLast << " width last: " <<sz1.width<< " topleft penultimate: " <<subsliceList[j-1].sub_slice_topleft.xCoordinate<<" topleft last: " << subsliceList[j].sub_slice_topleft.xCoordinate<< endl;

					if(xcoordinateLast!=sz1.width && xcoordinateLast >0){
						//cut to get only the different information of the image (remove the repeated image part)
						Mat subslicecut;
						int s1 = xcoordinateLast;
						int s2 = sz1.width;
						subslicecut = Mat(subslice2, Range(0,sz1.height), Range(s1,s2)).clone();

						//fill with 0s the remaining size of the subslice (minimum size is the overlap)
						Mat blackImage = Mat::zeros(Size(sz1.width-subslicecut.cols,sz1.height),subslicecut.type());
						//Mat sliceLarge;
						hconcat(subslicecut,blackImage,subslice2);
					}
					//merge with the correct subslice image
					hconcat(slice_op,subslice2,slicedone);
					col_offset = subsliceList[j-2].sub_slice_topleft.xCoordinate;

					temp_slice.id = subslices_iteration;
					temp_slice.col_offset = col_offset;

					temp_slice.mergeTime = (getTickCount()-mergeTime)/getTickFrequency();
					sliceList.push_back(temp_slice);

					return slicedone;
				}

				hconcat(slice_op,subslice2,slicedone);
				col_offset = subsliceList[j-2].sub_slice_topleft.xCoordinate;

				temp_slice.id = subslices_iteration;
				temp_slice.col_offset = col_offset;
				temp_slice.mergeTime = (getTickCount()-mergeTime)/getTickFrequency();
				sliceList.push_back(temp_slice);

				return slicedone;
			}
			//last slice needs 3 or 2 subslices
			else if(subslices_iteration == cameraList.sub_slices_total-1)
			{
				int j=subslices_iteration;

				//more than 2 subslices received in total
				if(cameraList.sub_slices_total>2){

					Mat subslice0 = imdecode(subsliceList[j-2].data, CV_LOAD_IMAGE_GRAYSCALE);
					Mat subslice1 = imdecode(subsliceList[j-1].data, CV_LOAD_IMAGE_GRAYSCALE);
					Mat subslice2 = imdecode(subsliceList[j].data, CV_LOAD_IMAGE_GRAYSCALE);

					//in this case we need to cut the penultimate and antipenultimate in order to compute the last subslice
					Size sz1 = subslice1.size();
					Size sz2 = subslice2.size();

					//know where is the last column with the same information (image) of the penultimate subslice
					int xcoordinateLast = sz2.width-(subsliceList[j-1].sub_slice_topleft.xCoordinate+sz1.width-subsliceList[j].sub_slice_topleft.xCoordinate);
					cerr << "xcoordinateLast: "<< xcoordinateLast << " width last: "<<sz2.width<< " topleft penultimate: " <<subsliceList[j-1].sub_slice_topleft.xCoordinate<<" topleft last: " << subsliceList[j].sub_slice_topleft.xCoordinate<< endl;

					if(xcoordinateLast!=sz2.width && xcoordinateLast >0){
						//cut to get only the different information of the image (remove the repeated image part)
						Mat subslicecut2;
						int s1 = 0;
						int s2 = xcoordinateLast;
						subslicecut2 = Mat(subslice1, Range(0,sz2.height), Range(s1,s2)).clone();

						//get the width remaining part with the image of antipenultimate subslice
						Mat subslicecut1;
						s1 = xcoordinateLast;
						s2 = sz2.width;
						subslicecut1 = Mat(subslice0, Range(0,sz2.height), Range(s1,s2)).clone();

						//merge the firsts subslices to get the width
						hconcat(subslicecut1,subslicecut2,subslice1);

					}
					//merge with the last subslice
					hconcat(subslice1,subslice2,slicedone);

					col_offset = subsliceList[j-1].sub_slice_topleft.xCoordinate;

					temp_slice.id = subslices_iteration;
					temp_slice.col_offset = col_offset;
					temp_slice.last_subslices_iteration = true;
					cerr << "last subslice" << endl;
					temp_slice.mergeTime = (getTickCount()-mergeTime)/getTickFrequency();
					sliceList.push_back(temp_slice);

					return slicedone;
				}
				//only 2 subslices received in total
				else if(cameraList.sub_slices_total==2){
					//in this case we don't have the necessary information (subslices)
					Mat subslice0 = imdecode(subsliceList[j-1].data, CV_LOAD_IMAGE_GRAYSCALE);
					Mat subslice1 = imdecode(subsliceList[j].data, CV_LOAD_IMAGE_GRAYSCALE);

					//cut the penultimate in order to compute the last subslice
					//know where is the last column with the same information (image) of the penultimate subslice
					Size sz0 = subslice0.size();
					Size sz1 = subslice1.size();
					int xcoordinateLast = sz1.width-(subsliceList[j-1].sub_slice_topleft.xCoordinate+sz1.width-subsliceList[j].sub_slice_topleft.xCoordinate);
					cerr << "xcoordinateLast: "<< xcoordinateLast << " width last: " <<sz0.width<< " topleft penultimate: " <<subsliceList[j-1].sub_slice_topleft.xCoordinate<<" topleft last: " << subsliceList[j].sub_slice_topleft.xCoordinate<< endl;

					if(xcoordinateLast!=sz1.width && xcoordinateLast >0){
						//cut to get only the different information of the image (remove the repeated image part)
						Mat subslicecut;
						int s1 = 0;
						int s2 = xcoordinateLast;
						subslicecut = Mat(subslice0, Range(0,sz1.height), Range(s1,s2)).clone();

						//fill with 0s the remaining size of the subslice (minimum size is the overlap)
						Mat blackImage = Mat::zeros(Size(sz1.width-subslicecut.cols,sz1.height),subslicecut.type());
						hconcat(blackImage,subslicecut,subslice0);
					}
					hconcat(subslice0,subslice1,slicedone);

					col_offset = subsliceList[j-1].sub_slice_topleft.xCoordinate;

					temp_slice.id = subslices_iteration;
					temp_slice.col_offset = col_offset;
					temp_slice.last_subslices_iteration = true;
					temp_slice.mergeTime = (getTickCount()-mergeTime)/getTickFrequency();
					sliceList.push_back(temp_slice);

					return slicedone;
				}
			}
		}
	}
}

void ProcessingManager::Processing_thread_cooperator(int i){

	boost::mutex::scoped_lock lock(thread_mutex);
	while(processcond == false){
		thread_condition.wait(lock);
	}
	cout << "PM: I'm entering the Processing thread "<< i+1 << endl;

	while(1){

		subslices_iteration++;

		vector<subslice> subsliceListsave = getData(subslices_iteration);

		//merge subslices to a slice
		int f = subsliceListsave.size();

		mergeTime = getTickCount();
		Mat slice_merged = mergeSubSlices(subslices_iteration,subsliceListsave);
		mergeTime = (getTickCount()-mergeTime)/getTickFrequency();
		//start processing timer
		if(subslices_iteration==1){
			processingTime = getTickCount();
		}
		//process resulting slice and save keypoints/features
		node_manager->Processing_slice(i,subslices_iteration, slice_merged,cameraList.detection_threshold, cameraList.max_features);
		slice_merged.release();
		subsliceListsave.clear();

	}

}



void ProcessingManager::storeKeypointsAndFeatures(int subslices_iteration_,vector<KeyPoint>& kpts,Mat& features,
		double detTime, double descTime){

	int i = subslices_iteration_-1;

	//save features
	features_buffer.push_back(features);

	//sum parameters
	cameraList.detTime += detTime;
	cameraList.descTime += descTime;
	cameraList.mergeTime += sliceList[i].mergeTime;
	cameraList.kptsSize += kpts.size();

	//save keypoints
	for(int j=0;j<kpts.size();j++){
		kpts[j].pt.x = kpts[j].pt.x + sliceList[i].col_offset;
		keypoint_buffer.push_back(kpts[j]);
	}
	cerr << "subslice_iteration " << subslices_iteration << "last subslices iteration? " << sliceList[i].last_subslices_iteration << endl;
	//if last slice in the Coop, do an average of the parameters and send to camera
	if(sliceList[i].last_subslices_iteration==true){

		//end processing timer
		processingTime = (getTickCount()-processingTime)/getTickFrequency();

			int check_id, check_col_offset;
			double check_mergeTime;

			std::ofstream out;
			if(node_manager->node_id == 3)
				out.open("checkP3.txt", std::ios::app);
			else if(node_manager->node_id == 4)
				out.open("checkP4.txt", std::ios::app);
			else
				out.open("checkP5.txt", std::ios::app);
			for(int g=0;g<sliceList.size();g++){
				check_id = sliceList[g].id;
				check_col_offset = sliceList[g].col_offset;
				check_mergeTime = sliceList[g].mergeTime;
				out << check_id << "	" << check_col_offset << "	" << check_mergeTime << endl;
			}
			out << "-----------------------" << endl;
			out.close();

		//clear sliceList and subsliceList
		sliceList.clear();
		subsliceList.clear();
		count_subslices=0;

		last_subslice_received=false;
		dataavailable=0;
		subslices_iteration=0;

		//encode kpts/features and send DataATCmsg to Camera
		node_manager->notifyCooperatorCompleted(cameraList.id,keypoint_buffer,features_buffer,cameraList.detTime,cameraList.descTime, cameraList.mergeTime, processingTime, cameraList.connection);

		features_buffer.release();
		keypoint_buffer.clear();

		cout << "PM: Coop from Camera " << cameraList.id << " finished" << endl;
	}

}

void ProcessingManager::setData(){

	mutex.lock();
	dataavailable++;
	mutex.unlock();
}

vector<subslice> ProcessingManager::getData(int subslices_iteration){

	mutex.lock();
	while(dataavailable==0){
		mutex.unlock();
		mutex.lock();
	}
	vector<subslice> subsliceListdone;

	//first slice need 2 subslices to start processing
	if(cameraList.slice_id == 1){
		if(subslices_iteration==1){
			while(dataavailable<2){
				mutex.unlock();
				mutex.lock();
			}
			dataavailable--;
			dataavailable--;
			subsliceListdone = subsliceList;
		}else if(cameraList.slices_total == 1 && subslices_iteration == cameraList.sub_slices_total-1){ //one coop only, last subslice
			dataavailable--;
			dataavailable++;
			subsliceListdone = subsliceList;
		}else if(subslices_iteration>1){
			while(dataavailable==0){
				mutex.unlock();
				mutex.lock();
			}
			dataavailable--;
			subsliceListdone = subsliceList;
		}
	}
	//last slice needs 3 or 2 subslices to start processing
	else if(cameraList.slices_total == cameraList.slice_id){

		if(cameraList.sub_slices_total==2){ //2 subslices in total only
			while(dataavailable<2){
				mutex.unlock();
				mutex.lock();
			}
			dataavailable--;
			dataavailable--;
			subsliceListdone = subsliceList;
		}else if(subslices_iteration==cameraList.sub_slices_total){ //2 subslices in total only
			while(dataavailable<2){
				mutex.unlock();
				mutex.lock();
			}
			dataavailable--;
			subsliceListdone = subsliceList;
		}else if(subslices_iteration==1){ //3 subslices or more
			while(dataavailable<3){
				mutex.unlock();
				mutex.lock();
			}
			dataavailable--;
			dataavailable--;
			dataavailable--;
			subsliceListdone = subsliceList;
			if(subslices_iteration+2==cameraList.sub_slices_total){
				dataavailable++;
				subsliceListdone = subsliceList;
			}
		}else if(subslices_iteration>1){ //remaining subslices
			while(dataavailable==0){
				mutex.unlock();
				mutex.lock();
			}
			dataavailable--;
			subsliceListdone = subsliceList;
			if(subslices_iteration+2==cameraList.sub_slices_total){
				dataavailable++;
				subsliceListdone = subsliceList;
			}
		}
	}
	//other slices need 3 subslices to start processing
	else{
		if(subslices_iteration==1){ //3 subslices or more
			while(dataavailable<3){
				mutex.unlock();
				mutex.lock();
			}
			dataavailable--;
			dataavailable--;
			dataavailable--;
			subsliceListdone = subsliceList;
		}else if(subslices_iteration>1){ //remaining subslices
			while(dataavailable==0){
				mutex.unlock();
				mutex.lock();
			}
			dataavailable--;
			subsliceListdone = subsliceList;
		}
	}
	mutex.unlock();
	return subsliceListdone;
}
