/*
 * OffloadingManager.cpp
 *
 *  Created on: 19/set/2014
 *      Author: greeneyes
 */

#include "OffloadingManager.h"

#include <iostream>
#include <fstream>

bool CtcoefComp(cooperator i, cooperator j){
	return (i.Ctcoef < j.Ctcoef);
}

void OffloadingManager::addCooperator(Connection* c){
	cooperator temp_coop;
	temp_coop.connection = c;
	temp_coop.processing_speed_estimator = new ProcessingSpeedEstimator();
	temp_coop.tx_speed_estimator = new TxSpeedEstimator();
	temp_coop.processing_time_coef = new ProcessingCoef();
	temp_coop.transmission_time_coef = new TransmissionCoef();
	//Set initial values for the parameters:
	temp_coop.bandwidth = 20e6;
	temp_coop.Pdpx = 350000; //250000
	temp_coop.Pdip = 1000; //1500
	temp_coop.Pe = 400; //250
	temp_coop.Pm = 400; //250000
	
	//Ptcoef*(1+alpha_d) >= S*Ctcoef
	temp_coop.Ctcoef = 0.0000003; //0.0003
	temp_coop.Ptcoef = 0.000003; //0.0000002
	temp_coop.alpha_d = 1000; //3520
	
	//ALEXIS 09/01 COOP ID
	int m=2;
	std::pair<std::set<int>::iterator,bool> ret;
	while(true){
		m++;
		ret = id.insert(m);
		if (ret.second==true){
			break;
		}
	}
	temp_coop.id = m;
	std::cerr << "added Coop " << temp_coop.id << endl;
	//

	cooperatorList.push_back(temp_coop);
}

void OffloadingManager::removeCooperator(Connection* c){
	for(int i=0;i<cooperatorList.size();i++){
		cooperator temp_coop = cooperatorList[i];
		if(temp_coop.connection == c){
			delete temp_coop.processing_speed_estimator;
			delete temp_coop.tx_speed_estimator;
			//ALEXIS 09/01 COOP ID
			std::cerr << "removed Coop " << temp_coop.id << endl;
			int m = temp_coop.id;
			id.erase(m);
			//
			cooperatorList.erase(cooperatorList.begin()+i);
		}
	}
}

Mat OffloadingManager::computeLoads(Mat& image){
	vector<double> c;
	vector<double> p;
	vector<double> alphad;

	sortCooperators();
	
	for(int i=0;i<cooperators_to_use;i++){
		c.push_back(cooperatorList[i].Ctcoef);
		p.push_back(cooperatorList[i].Ptcoef);
		alphad.push_back(cooperatorList[i].alpha_d);
	}
	
	//double overlap = OVERLAP;
	double overlap = (double)168.0/(2*image.cols);
	overlap_normalized = image.cols*image.rows*overlap;
	algorithms.SetImageParameters(image.cols, image.rows, overlap);
	width_=image.cols;
	height_=image.rows;
	overlap_=overlap;
	
	//Solve:
	double lpsolveTime = getTickCount();
	algorithms.CutVectorOptimization(cooperators_to_use, c, p, alphad);
	lpsolveTime = (getTickCount()-lpsolveTime)/getTickFrequency();
	std::cerr << "lpsolveTime = " << lpsolveTime << "sec\n";
	vector<int> cutvector = algorithms.getCutVector();

	for(size_t j=0; j<cutvector.size(); j++){
		std::cerr << "cutvector: " << cutvector[j] << std::endl;
	}
	std::cerr << "Estimated completion time: " << algorithms.getCompletionTime() << "sec\n";

	//if cuts=0, have to use less cooperators
	while(cutvector[0] == 0){
		cooperators_to_use = cooperators_to_use-1;
		for(int j=0;j<cutvector.size();j++){
			cutvector[j]=cutvector[j+1];
		}
	}
	cutvector.resize(cooperators_to_use);

	/*//save assignment position
	asignmentvector.resize(cooperators_to_use);
	for(int j=0; j<cooperators_to_use; j++){
		asignmentvector[j] = cooperatorList[j].id;
	}*/

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
		algorithms.setInitialDetectionThreshold(INITIAL_DETECTION_THRESHOLD);
	}
	msg->setDetectorThreshold(next_detection_threshold);
	msg->setMaxNumFeat(400*1.1); //ALEXIS SIMULATION 03/02
	msg->setSource(node_manager->node_id); //ALEXIS 11/12 can be changed by node_id
	for(int i=0;i<cooperatorList.size();i++){ //ORIGINAL
	//for(int i=0;i<cooperators_to_use;i++){ //ALEXIS 11/12 -> to not sent multiple StartDATCMsg unnecessary. 14/12 not working correctly
		//msg->setDestination(i+3); //ALEXIS 11/12
		msg->setDestination(cooperatorList[i].id); //ALEXIS 09/01 COOP ID
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
	start_time_global = getTickCount();
	cerr << "start timer: " << start_time_global << " ticks ///////////////////////////////////////////////////////////" <<endl;
	start_time = getTickCount();
	transmitNextCoop();
}

int OffloadingManager::probeLinks(){
	return 0;
	/*for(int i=0;i < cooperatorList.size();i++){
		//DUMMY BANDWIDTH - REPLACE WITH ACTUAL PROBING
		cooperatorList[i].bandwidth = 24000000;
		cooperatorList[i].CPUspeed = 1480000;
	}
	return 0;*/

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
	algorithms.SetTargetKeypoints(target_num_keypoints);

	//here we should start a timer that will check if data is received from all cooperators
	//if it expires, it should notify the node_manager anyway to prevent deadlocks.
//TODO
//	t.expires_from_now(boost::posix_time::seconds(5));
//	t.async_wait(boost::bind(&OffloadingManager::timerExpired, this, boost::asio::placeholders::error));
}

double OffloadingManager::getNextDetectionThreshold() {
	return next_detection_threshold;
}

void OffloadingManager::estimate_parameters(cooperator* coop, int i) {
	//Processing parameters
	int ret = coop->processing_speed_estimator->AddObservation(coop->detTime, coop->descTime, coop->mergeTime, coop->Npixels, coop->Nkeypoints);
	if(ret==0){
		coop->Pdpx = coop->processing_speed_estimator->getPdpx();
		coop->Pdip = coop->processing_speed_estimator->getPdip();
		coop->Pm = coop->processing_speed_estimator->getPm();
		if(coop->Nkeypoints > 0) coop->Pe = coop->processing_speed_estimator->getPe();
	}
	//alpha_d
	double alpha_d = coop->processing_time_coef->setAlphad(coop->Pdpx, coop->Pdip, coop->Pe, coop->Pm);
	coop->alpha_d = alpha_d;

	//Transmission time Coef
	coop->transmission_time_coef->AddObservation(coop->txTime, coop->Npixels);
	coop->Ctcoef = coop->transmission_time_coef->getTransmissionTimeCoef();
	//coop->bandwidth = 8*coop->Npixels/coop->txTime; //FIXME Only for bmp encoding, 8bits per pixel


	//Processing time Coef
	bool double_overlap = true;
	if (i==0 || i == cooperators_to_use-1){
		double_overlap = false;
	}
	coop->processing_time_coef->setOverlap(overlap_normalized, double_overlap);
	coop->processing_time_coef->AddObservation(coop->processingTime, coop->Npixels, coop->Nkeypoints, alpha_d);
	coop->Ptcoef = coop->processing_time_coef->getProcessingTimeCoef();
	
	/////WITHOUT BBONES
	//coop->Ctcoef = 3.70e-6;
	//coop->Ptcoef = 6.40e-6;

	std::cerr << " Node: " << coop << std::endl;
	std::cerr << "estimate_processing_parameters: detTime=" << coop->detTime << "\tdescTime=" << coop->descTime << "\tNpix=" << coop->Npixels << "\tNkp=" << coop->Nkeypoints << "\n";
	std::cerr << "estimate_processing_parameters: Pdpx=" << coop->Pdpx << "\tPdip=" << coop->Pdip << "\tPe=" << coop->Pe << "\tPm" << coop->Pm << "\n";
	std::cerr << "estimate_processing_parameters: estimated Ptcoef= " << coop->Ptcoef << "\n";
	std::cerr << "estimate_processing_parameters: processingTime= " << coop->processingTime << "\tNpix=" << coop->Npixels << "\tNkp=" << coop->Nkeypoints << "\n";
	std::cerr << "txTime: " << coop->txTime << "\testimated Ctcoef: " << coop->Ctcoef << "sec/pixel" << std::endl;
	std::cerr << "cooperator completion time:" << coop->completionTime << std::endl;
	std::cerr << "idleTime + txTime + processingTime = " << coop->idleTime + coop->txTime + coop->processingTime  << std::endl;
	
		std::ofstream out;
		if(coop->id == 3)
			out.open("P3.txt", std::ios::app);
		else if(coop->id == 4)
			out.open("P4.txt", std::ios::app);	
		else if(coop->id == 5)
			out.open("P5.txt", std::ios::app);
		out << coop->Ptcoef << " " << coop->Ctcoef << " " << coop->Npixels << " " << coop->Npixelssubslices<< " " << coop->Nkeypoints << " Tidle: " << coop->idleTime << " Tx: " << coop->txTime << " Tm: " << coop->mergeTime << " Tp: " << coop->processingTime << " Tc: "<< coop->completionTime << " alpha: " << alpha_d << std::endl;
		out.close();
		
}

void OffloadingManager::sortCooperators()
{
	std::sort(cooperatorList.begin(), cooperatorList.end(), CtcoefComp);
}

void OffloadingManager::addKeypointsAndFeatures(vector<KeyPoint>& kpts,Mat& features, Connection* cn,
		double detTime, double descTime, double mergeTime, double processingTime){

	mut.lock();
	features_buffer.push_back(features);

	if(cn){
		for(int i=0;i<cooperatorList.size();i++){
			if(cn == cooperatorList[i].connection){
				//add time measurements
				cooperatorList[i].completionTime = (getTickCount()-start_time)/getTickFrequency();
				cooperatorList[i].detTime = detTime;
				cooperatorList[i].descTime = descTime;
				cooperatorList[i].mergeTime = mergeTime;
				//cooperatorList[i].fencTime = fencTime;
				cooperatorList[i].processingTime = processingTime;
				//compensate for slicing if keypoints come from a cooperator
				for(int j=0;j<kpts.size();j++){
					kpts[j].pt.x = kpts[j].pt.x + cooperatorList[i].col_offset;
					keypoint_buffer.push_back(kpts[j]);
				}
				cooperatorList[i].Nkeypoints = kpts.size();
				estimate_parameters(&cooperatorList[i],i);
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
	if(received_cooperators == cooperators_to_use+1){ //ALEXIS MUTEX PROBLEM 03/02
		//data received from all cooperators: stop timer
		completionTimeGlobal = (getTickCount()-start_time_global)/getTickFrequency();
		cerr << "start timer: " << completionTimeGlobal << " ticks ///////////////////////////////////////////////////////////" <<endl;
		std::cerr << "Total Completion Time: " << completionTimeGlobal << "\n";
		
		vector<int> cutvector = algorithms.getCutVector();

		std::ofstream out;
			if(node_manager->node_id == 1)
				out.open("completionTimeGlobalC1.txt", std::ios::app);
			else
				out.open("completionTimeGlobalC2.txt", std::ios::app);
			out << "Real: "<< completionTimeGlobal << "	Estimated: "<< algorithms.getCompletionTime() << "	";
			for(size_t j=0; j<cutvector.size(); j++){
				out << " "<< cooperatorList[j].id << " -> cut: " << cutvector[j] << " ";
			}
			out << std::endl;
			out.close();
			
		t.cancel();

		algorithms.AddKeypoints(keypoint_buffer);
		//get next detection threshold
		next_detection_threshold = algorithms.GetNextDetectionThreshold();

std::cerr << "Next detection threshold: " << next_detection_threshold << "\n";
std::cerr << "Added " << keypoint_buffer.size() << " keypoints\n";
//printKeypoints(keypoint_buffer);

		node_manager->notifyOffloadingCompleted(keypoint_buffer,features_buffer,camDetTime,camDescTime);
	}
	mut.unlock();
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
		param[1] = 70;

		cooperatorList[i].idleTime = (getTickCount()-start_time)/getTickFrequency();
		cooperatorList[i].txTime = getTickCount();
		
		transmitNextSlice(i);

		next_coop++;
	}
}
void OffloadingManager::transmitNextSlice(int i){
	
	vector<uchar> bitstream;
	vector<int> param = vector<int>(2);
	param[0] = CV_IMWRITE_JPEG_QUALITY;
	param[1] = 70;
	cooperatorList[i].Npixelssubslices = 0;
	
	vector<cooperator> cooperatorSlice;
	cooperator opt;
	int sub_slices_total = (int)ceil((cooperatorList[i].image_slice.cols)/(width_*overlap_));

	for(int j=0;j<sub_slices_total;j++){
		cooperatorSlice.push_back(opt);
	}

	int s1, s2, offset;

/*
	if(i==0){ //first slice

		//First subslice
		s1 = 0;
		s2 = (int)ceil(overlap_*width_);
		cooperatorSlice[0].image_slice = Mat(cooperatorList[i].image_slice, Range(0,height_), Range(s1,s2)).clone();
		cooperatorSlice[0].col_offset = s1;
		cooperatorSlice[0].Npixels = cooperatorSlice[0].image_slice.rows * cooperatorSlice[0].image_slice.cols;

		for(int j=1; j<sub_slices_total-1; j++){
		//Middle subslices
			s1 = (int)floor(j*overlap_*width_);
			s2 = (int)ceil((j+1)*overlap_*width_);
			cooperatorSlice[j].image_slice = Mat(cooperatorList[i].image_slice, Range(0,height_), Range(s1,s2)).clone();
			cooperatorSlice[j].col_offset = s1;
			cooperatorSlice[j].Npixels = cooperatorSlice[i].image_slice.rows * cooperatorSlice[i].image_slice.cols;
		}
	}
	else if(i!=0){ //no first slice

		//First subslice
		//offset = max(0,(int)floor(cooperatorList[i-1].image_slice.cols-2*overlap_*width_));
		s1 = (int)floor(cooperatorList[i-1].image_slice.cols-overlap_*width_);
		s2 = cooperatorList[i-1].image_slice.cols;
		cooperatorSlice[0].image_slice = Mat(cooperatorList[i].image_slice, Range(0,height_), Range(s1,s2)).clone();
		cooperatorSlice[0].col_offset = s1;
		cooperatorSlice[0].Npixels = cooperatorSlice[0].image_slice.rows * cooperatorSlice[0].image_slice.cols;

		for(int j=1; j<sub_slices_total-1; j++){
			//Middle subslices
			//offset = max(0, (int)floor(cooperatorList[i-1].image_slice.cols+(j-2)*overlap_*width_));
			s1 = (int)floor(cooperatorList[i-1].image_slice.cols+j*overlap_*width_);
			s2 = (int)ceil(cooperatorList[i-1].image_slice.cols+(j+1)*overlap_*width_);
			cooperatorSlice[j].image_slice = Mat(cooperatorList[i].image_slice, Range(0,height_), Range(s1,s2)).clone();
			cooperatorSlice[j].col_offset = s1;
			cooperatorSlice[j].Npixels = cooperatorSlice[j].image_slice.rows * cooperatorSlice[j].image_slice.cols;

			}
	}
	//Last subslice
	s1 = (int)floor(cooperatorList[i].image_slice.cols-overlap_*width_);
	s2 = cooperatorList[i].image_slice.cols;
	cooperatorSlice[sub_slices_total-1].image_slice = Mat(cooperatorList[i].image_slice, Range(0,height_), Range(s1,s2)).clone();
	cooperatorSlice[sub_slices_total-1].col_offset = s1;
	cooperatorSlice[sub_slices_total-1].Npixels = cooperatorSlice[sub_slices_total-1].image_slice.rows * cooperatorSlice[sub_slices_total-1].image_slice.cols;
*/
			
	//First subslice
	s1 = 0;
	s2 = min(width_, (int)ceil(overlap_*width_));
	cooperatorSlice[0].image_slice = Mat(cooperatorList[i].image_slice, Range(0,height_), Range(s1,s2)).clone();
	cooperatorSlice[0].col_offset = s1;
	cooperatorSlice[0].Npixels = cooperatorSlice[0].image_slice.rows * cooperatorSlice[0].image_slice.cols;

	for(int j=1; j<sub_slices_total-1; j++){
	//Middle subslices
		s1 = max(0, (int)floor(j*overlap_*width_));
		s2 = min(width_, (int)ceil((j+1)*overlap_*width_));
		cooperatorSlice[j].image_slice = Mat(cooperatorList[i].image_slice, Range(0,height_), Range(s1,s2)).clone();
		cooperatorSlice[j].col_offset = s1;
		cooperatorSlice[j].Npixels = cooperatorSlice[j].image_slice.rows * cooperatorSlice[j].image_slice.cols;
	}
	//Last subslice
	s1 = max(0, (int)floor(cooperatorList[i].image_slice.cols-overlap_*width_));
	s2 = cooperatorList[i].image_slice.cols;
	cooperatorSlice[sub_slices_total-1].image_slice = Mat(cooperatorList[i].image_slice, Range(0,height_), Range(s1,s2)).clone();
	cooperatorSlice[sub_slices_total-1].col_offset = s1;
	cooperatorSlice[sub_slices_total-1].Npixels = cooperatorSlice[sub_slices_total-1].image_slice.rows * cooperatorSlice[sub_slices_total-1].image_slice.cols;


	for(int z=0;z<sub_slices_total;z++){
		
		double enc_time = getTickCount();
		if(COMPRESS_IMAGE == 1){
			imencode(".jpg",cooperatorSlice[z].image_slice,bitstream,param);
		}else{
			imencode(".bmp", cooperatorSlice[z].image_slice,bitstream);
		}
		enc_time = (getTickCount()-enc_time)/getTickFrequency();
		Coordinate_t top_left;
		top_left.xCoordinate = cooperatorSlice[z].col_offset;
		top_left.yCoordinate = 0;
		
		cooperatorList[i].Npixelssubslices +=cooperatorSlice[z].Npixels;

		DataCTAMsg *msg = new DataCTAMsg(i,sub_slices_total,top_left,bitstream.size(),enc_time,0,bitstream);
		//ALEXIS 11/12
		msg->setSource(node_manager->node_id);
		//msg->setDestination(i+3);
		//
		msg->setDestination(cooperatorList[i].id); //ALEXIS 09/01 COOP ID
		cooperatorList[i].connection->writeMsg(msg);
	}	
	cooperatorSlice.clear();
}


/*void OffloadingManager::printKeypoints(vector<KeyPoint>& kpts) {
	vector<KeyPoint> buf = kpts;
	std::sort(buf.begin(), buf.end(), greater_than_response());
	for(size_t i=0; i<buf.size(); i++){
		std::cerr << buf[i].response << "\t(" << buf[i].pt.x << "," <<  buf[i].pt.y << ")\n";
	}
 }*/

void OffloadingManager::notifyACKslice(int frameID, Connection* cn) {
	mut.lock();
	for(int i=0;i<cooperatorList.size();i++){
		if(cn == cooperatorList[i].connection){
			cerr << "OM: TxTime ended for Coop " << cooperatorList[i].id << endl;
			cooperatorList[i].txTime = (getTickCount()-cooperatorList[i].txTime)/getTickFrequency();
		}
	}
	transmitNextCoop();
	mut.unlock();
}
