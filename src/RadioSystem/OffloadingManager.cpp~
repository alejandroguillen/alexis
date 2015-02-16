/*
 * OffloadingManager.cpp
 *
 *  Created on: 19/set/2014
 *      Author: greeneyes
 */

#include "OffloadingManager.h"
//#include <algorithm>

bool bandwidthComp(cooperator i, cooperator j){
	return (i.bandwidth > j.bandwidth);
}

void OffloadingManager::addCooperator(Connection* c){
	cooperator temp_coop;
	temp_coop.connection = c;
	temp_coop.processing_speed_estimator = new ProcessingSpeedEstimator();
	temp_coop.tx_speed_estimator = new TxSpeedEstimator();
	//Set initial values for the parameters:
	temp_coop.bandwidth = 20e6;
	temp_coop.Pdpx = 3.2e6;
	temp_coop.Pdip = 10000;
	temp_coop.Pe = 1000;

	cooperatorList.push_back(temp_coop);
}

void OffloadingManager::removeCooperator(Connection* c){
	for(int i=0;i<cooperatorList.size();i++){
		cooperator temp_coop = cooperatorList[i];
		if(temp_coop.connection == c){
			delete temp_coop.processing_speed_estimator;
			delete temp_coop.tx_speed_estimator;
			cooperatorList.erase(cooperatorList.begin()+i);
		}
	}
}


Mat OffloadingManager::computeLoads(Mat& image){
	vector<double> c;
	vector<double> pdpx;
	vector<double> pdip;
	vector<double> pe;

	sortCooperators();
	for(int i=0;i<cooperators_to_use;i++){
		c.push_back(8*1.0/(cooperatorList[i].bandwidth)); //TODO Check bandwidth units. c should be seconds/bit
		pdpx.push_back(cooperatorList[i].Pdpx);
		pdip.push_back(cooperatorList[i].Pdip);
		pe.push_back(cooperatorList[i].Pe);
	}

	//double overlap = OVERLAP;
	double overlap = (double)168.0/(2*image.cols);
	loadbalancing.SetImageParameters(image.cols, image.rows, overlap);
	//Solve:
	double lpsolveTime = getTickCount();
	loadbalancing.CutVectorOptimization(cooperators_to_use, c, pdpx, pdip, pe);
	lpsolveTime = (getTickCount()-lpsolveTime)/getTickFrequency();
	std::cerr << "lpsolveTime = " << lpsolveTime << "sec\n";
	vector<int> cutvector = loadbalancing.getCutVector();

	for(size_t j=0; j<cutvector.size(); j++){
		std::cerr << "cutvector: " << cutvector[j] << std::endl;
	}
	std::cerr << "Estimated completion time: " << loadbalancing.getCompletionTime() << "sec\n";

	//Set loads in cooperatorList
	int s1, s2;
	if(cooperators_to_use == 1){
		s1 = 0;
		s2 = image.cols;
		cooperatorList[0].image_slice = Mat(image, Range(0,image.rows), Range(s1,s2)).clone();
		cooperatorList[0].col_offset = s1;
		cooperatorList[0].Npixels = cooperatorList[0].image_slice.rows * cooperatorList[0].image_slice.cols;
	}
	else{
		//First cooperator
		s1 = 0;
		s2 = min(image.cols, (int)ceil(cutvector[0]+overlap*image.cols));
		cooperatorList[0].image_slice = Mat(image, Range(0,image.rows), Range(s1,s2)).clone();
		cooperatorList[0].col_offset = s1;
		cooperatorList[0].Npixels = cooperatorList[0].image_slice.rows * cooperatorList[0].image_slice.cols;

		//Middle cooperators
		for(int i=1; i<cooperators_to_use-1; i++){
			s1 = max(0, (int)floor(cutvector[i-1]-overlap*image.cols));
			s2 = min(image.cols, (int)ceil(cutvector[i]+overlap*image.cols));
			cooperatorList[i].image_slice = Mat(image, Range(0,image.rows), Range(s1,s2)).clone();
			cooperatorList[i].col_offset = s1;
			cooperatorList[i].Npixels = cooperatorList[i].image_slice.rows * cooperatorList[i].image_slice.cols;
		}

		//Last cooperator
		s1 = max(0, (int)floor(cutvector[cooperators_to_use-2]-overlap*image.cols));
		s2 = image.cols;
		cooperatorList[cooperators_to_use-1].image_slice = Mat(image, Range(0,image.rows), Range(s1,s2)).clone();
		cooperatorList[cooperators_to_use-1].col_offset = s1;
		cooperatorList[cooperators_to_use-1].Npixels = cooperatorList[cooperators_to_use-1].image_slice.rows * cooperatorList[cooperators_to_use-1].image_slice.cols;
	}

	//No load for camera
	Mat myLoad = Mat(image, Range(0,0), Range(0,0)).clone();
	return myLoad;
}

void OffloadingManager::transmitStartDATC(StartDATCMsg* msg){
	if(next_detection_threshold==0){
		//First frame, use INITIAL_DETECTION_THRESHOLD
		next_detection_threshold = INITIAL_DETECTION_THRESHOLD;
		loadbalancing.setInitialDetectionThreshold(INITIAL_DETECTION_THRESHOLD);
	}
	msg->setDetectorThreshold(next_detection_threshold);
	msg->setMaxNumFeat((msg->getMaxNumFeat())*1.1);

	for(int i=0;i<cooperatorList.size();i++){
		cooperatorList[i].connection->writeMsg(msg);
	}
	delete(msg);
}

/*void OffloadingManager::transmitLoads(){
	vector<uchar> bitstream;
	vector<int> param = vector<int>(2);
	param[0] = CV_IMWRITE_JPEG_QUALITY;
	param[1] = 100;

	for(int i=0;i<cooperators_to_use;i++){
		double enc_time = getTickCount();
		imencode(".jpg",cooperatorList[i].image_slice,bitstream,param);
//		imencode(".bmp", cooperatorList[i].image_slice,bitstream);
		enc_time = (getTickCount()-enc_time)/getTickFrequency();
		Coordinate_t top_left;
		top_left.xCoordinate = cooperatorList[i].col_offset;
		top_left.yCoordinate = 0;

		if(i==0){
			start_time = getTickCount();
		}

		DataCTAMsg *msg = new DataCTAMsg(0,1,top_left,bitstream.size(),enc_time,0,bitstream);
		cooperatorList[i].txTime = getTickCount();
		cooperatorList[i].connection->writeMsg(msg);
	}
}*/

void OffloadingManager::transmitLoads(){
	next_coop = 0;
	start_time = getTickCount();
	transmitNextCoop();
}

int OffloadingManager::probeLinks(){
	for(int i=0;i < cooperatorList.size();i++){
		//DUMMY BANDWIDTH - REPLACE WITH ACTUAL PROBING
		cooperatorList[i].bandwidth = 24000000;
		cooperatorList[i].CPUspeed = 1480000;
	}
	return 0;

	/*char command[60];
	for(int i=0;i<cooperatorList.size();i++){
		strcpy(command,"iperf -c");
		strcat(command,inet_ntoa(cooperatorList[i].client.sin_addr));
		strcat(command," -t 5 -p 8000  >/home/ubuntu/iperfLog.txt");
		cout << "Starting bandwidth test: " << inet_ntoa(cooperatorList[i].client.sin_addr) << endl;
		system(command);

		//Parsing file to get bandwidth
		double bandwidth=getBandwidth(false);
		if(bandwidth==0)
			return -1;
		else{
			cooperatorList[i].bandwidth = bandwidth*1000000;
        		cooperatorList[i].CPUspeed = 1480000;
		}
	}

	return 0;*/
}

void OffloadingManager::createOffloadingTask(int num_cooperators, int target_num_keypoints) {
	received_cooperators = 0;
	cooperators_to_use = num_cooperators;
	features_buffer.release();
	keypoint_buffer.clear();
	loadbalancing.SetTargetKeypoints(target_num_keypoints);

	//here we should start a timer that will check if data is received from all cooperators
	//if it expires, it should notify the node_manager anyway to prevent deadlocks.
//TODO
//	t.expires_from_now(boost::posix_time::seconds(5));
//	t.async_wait(boost::bind(&OffloadingManager::timerExpired, this, boost::asio::placeholders::error));
}

double OffloadingManager::getNextDetectionThreshold() {
	return next_detection_threshold;
}

void OffloadingManager::estimate_parameters(cooperator* coop) {
	//Processing parameters
	int ret = coop->processing_speed_estimator->AddObservation(coop->detTime, coop->descTime, coop->Npixels, coop->Nkeypoints);
	if(ret==0){
		coop->Pdpx = coop->processing_speed_estimator->getPdpx();
		coop->Pdip = coop->processing_speed_estimator->getPdip();
		if(coop->Nkeypoints > 0) coop->Pe = coop->processing_speed_estimator->getPe();
	}

	//Bandwidth
	coop->tx_speed_estimator->AddObservation(coop->txTime, coop->Npixels);
	coop->bandwidth = coop->tx_speed_estimator->getBandwidth();
	//coop->bandwidth = 8*coop->Npixels/coop->txTime; //FIXME Only for bmp encoding, 8bits per pixel

	std::cerr << " Node: " << coop << std::endl;
	std::cerr << "estimate_processing_parameters: detTime=" << coop->detTime << "\tdescTime=" << coop->descTime << "\tNpix=" << coop->Npixels << "\tNkp=" << coop->Nkeypoints << "\n";
	std::cerr << "estimate_processing_parameters: Pdpx=" << coop->Pdpx << "\tPdip=" << coop->Pdip << "\tPe=" << coop->Pe << "\n";
	std::cerr << "txTime: " << coop->txTime << "\testimated_bandwidth: " << coop->bandwidth << "bits/sec" << std::endl;
	std::cerr << "cooperator completion time:" << coop->completionTime << std::endl;
	std::cerr << "idleTime + txTime + detTime + descTime = " << coop->idleTime + coop->txTime + coop->detTime + coop->descTime  << std::endl;
}

void OffloadingManager::sortCooperators()
{
	std::sort(cooperatorList.begin(), cooperatorList.end(), bandwidthComp);
}

void OffloadingManager::addKeypointsAndFeatures(vector<KeyPoint>& kpts,Mat& features, Connection* cn,
		double detTime, double descTime, double kencTime, double fencTime){
	features_buffer.push_back(features);

	if(cn){
		for(int i=0;i<cooperatorList.size();i++){
			if(cn == cooperatorList[i].connection){
				//add time measurements
				cooperatorList[i].completionTime = (getTickCount()-start_time)/getTickFrequency();
				cooperatorList[i].detTime = detTime;
				cooperatorList[i].descTime = descTime;
				cooperatorList[i].kencTime = kencTime;
				cooperatorList[i].fencTime = fencTime;
				//compensate for slicing if keypoints come from a cooperator
				for(int j=0;j<kpts.size();j++){
					kpts[j].pt.x = kpts[j].pt.x + cooperatorList[i].col_offset;
					keypoint_buffer.push_back(kpts[j]);
				}
				cooperatorList[i].Nkeypoints = kpts.size();
				estimate_parameters(&cooperatorList[i]);
				break;
			}
		}
	}
	else{
	/*	//add time measurements
		camDetTime = detTime;
		camDescTime = descTime;
		camkEncTime = kencTime;
		camfEncTime = fencTime;

		for(int j=0;j<kpts.size();j++){
			keypoint_buffer.push_back(kpts[j]);
		}
	*/
	}

	//add
	received_cooperators++;
	if(received_cooperators == cooperators_to_use+1){
		//data received from all cooperators: stop timer
		t.cancel();

		loadbalancing.AddKeypoints(keypoint_buffer);
		//get next detection threshold
		next_detection_threshold = loadbalancing.GetNextDetectionThreshold();

std::cerr << "Next detection threshold: " << next_detection_threshold << "\n";
std::cerr << "Added " << keypoint_buffer.size() << " keypoints\n";
//printKeypoints(keypoint_buffer);

		node_manager->notifyOffloadingCompleted(keypoint_buffer,features_buffer,camDetTime,camDescTime);
	}
}

void OffloadingManager::timerExpired(const boost::system::error_code& error) {
	//check the errorcode:
	if(error != boost::asio::error::operation_aborted){
		cout << "Offloading timer expired" << endl;
		node_manager->notifyOffloadingCompleted(keypoint_buffer,features_buffer,camDetTime,camDescTime);
	}
	else
		cout << "Data received, canceling timer" << endl;
}

void OffloadingManager::startTimer() {
	r_thread = boost::thread(&OffloadingManager::runThread, this);
}

void OffloadingManager::runThread() {
	io.run();
	cout << "out of io service" << endl;
}

int OffloadingManager::getNumAvailableCoop() {
	return cooperatorList.size();
}

void OffloadingManager::transmitNextCoop() {
	if(next_coop < cooperators_to_use){
		int i = next_coop;
		vector<uchar> bitstream;
		vector<int> param = vector<int>(2);
		param[0] = CV_IMWRITE_JPEG_QUALITY;
		param[1] = 100;

		cooperatorList[i].idleTime = (getTickCount()-start_time)/getTickFrequency();
		cooperatorList[i].txTime = getTickCount();
		double enc_time = getTickCount();
		if(COMPRESS_IMAGE == 1){
			imencode(".jpg",cooperatorList[i].image_slice,bitstream,param);
		}else{
			imencode(".bmp", cooperatorList[i].image_slice,bitstream);
		}
		enc_time = (getTickCount()-enc_time)/getTickFrequency();
		Coordinate_t top_left;
		top_left.xCoordinate = cooperatorList[i].col_offset;
		top_left.yCoordinate = 0;

		DataCTAMsg *msg = new DataCTAMsg(0,1,top_left,bitstream.size(),enc_time,0,bitstream);
		cooperatorList[i].connection->writeMsg(msg);

		next_coop++;
	}
}

/*void OffloadingManager::printKeypoints(vector<KeyPoint>& kpts) {
	vector<KeyPoint> buf = kpts;
	std::sort(buf.begin(), buf.end(), greater_than_response());
	for(size_t i=0; i<buf.size(); i++){
		std::cerr << buf[i].response << "\t(" << buf[i].pt.x << "," <<  buf[i].pt.y << ")\n";
	}
 }*/

void OffloadingManager::notifyACKslice(int frameID, Connection* cn) {
	for(int i=0;i<cooperatorList.size();i++){
		if(cn == cooperatorList[i].connection){
			cooperatorList[i].txTime = (getTickCount()-cooperatorList[i].txTime)/getTickFrequency();
		}
	}
	transmitNextCoop();
}
