#include "Tasks.h"

#include <DetDescParams.h>
#include <ImageAcquisition.h>
#include <VisualFeatureExtraction.h>
#include <VisualFeatureEncoding.h>
#include <VisualFeatureDecoding.h>
#include "Messages/Header.h"
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "RadioSystem/OffloadingManager.h"
#include "RadioSystem/ProcessingManager.h"

void AcquireImageTask::execute(){
	cout << "executing the acquire_image_task" << endl;
	//imgAcq->takePicture(image);
	//image = imread("util/sample_VGA_1.jpg");
	image = imread("util/sample_VGA_3.jpg");
	cout << "ok!" << endl;
	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;
}


void SliceImageTask::execute(){
	int min_r = tl.y;
	int min_c = tl.x;
	int max_r = br.y;
	int max_c = br.x;
	slice = Mat(image, Range(min_r, max_r), Range(min_c, max_c)).clone();
	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;
}

void EncodeSliceTask::execute(){
	vector<int> param = vector<int>(2);
	param[0] = CV_IMWRITE_JPEG_QUALITY;
	param[1] = qualityFactor;
	encTime = getTickCount();
	imencode(".jpg",image,jpegBitstream,param);
	encTime = (getTickCount()-encTime)/getTickFrequency();
	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;
}

void DecodeSliceTask::execute(){
	decodedImage = imdecode(jpegBitstream,1);
	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;
}

void ExtractKeypointsTask::execute(){
	extractor->setDetThreshold("BRISK",det_thr);
	detTime = getTickCount();
	extractor->extractKeypoints(image,keypoints);
	detTime = (getTickCount()-detTime)/getTickFrequency();
	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;
}

void ExtractFeaturesTask::execute(){
	descTime = getTickCount();
	extractor->extractFeatures(image,keypoints,features);
	descTime = (getTickCount()-descTime)/getTickFrequency();
	extractor->cutFeatures(keypoints,features,max_features);
	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;
}

void EncodeKeypointsTask::execute(){
	if(method==0){ // dummy
		kencTime = getTickCount();
		encoder->dummy_encodeKeyPoints(keypoints,kptsBitstream);
		kencTime = (getTickCount()-kencTime)/getTickFrequency();

	}
	else{
		kencTime = getTickCount();
		encoder->encodeKeyPoints(keypoints,kptsBitstream,imWidth,imHeight,encodeAngles);
		kencTime = (getTickCount()-kencTime)/getTickFrequency();
	}

	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;

}

void EncodeFeaturesTask::execute(){

	if(method==0)// dummy
	{
		fencTime = getTickCount();
		encoder->dummy_encodeBinaryDescriptors(descName,
				features,
				featsBitstream);
		fencTime = (getTickCount()-fencTime)/getTickFrequency();

	}
	else // entropy coding
	{
		fencTime = getTickCount();
		encoder->encodeBinaryDescriptors(descName,
				features,
				featsBitstream);
		fencTime = (getTickCount()-fencTime)/getTickFrequency();
	}
	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;
}

void DecodeKeypointsTask::execute(){
	if(method == 0){
		decoder->dummy_decodeKeyPoints(kptsBitstream, keypoints);
		cout << "decoded " << keypoints.size() << " kpts cazzo" << endl;
	}
	else{
		decoder->decodeKeyPoints(kptsBitstream, keypoints,imWidth,imHeight,decodeAngles);
		cout << "decoded " << keypoints.size() << " kpts figa" << endl;

	}
	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;
}

void DecodeFeaturesTask::execute(){
	if(method==0)// dummy
	{
		decoder->dummy_decodeBinaryDescriptors(descName,
				featsBitstream,
				features);
	}
	else // entropy coding
	{
		decoder->decodeBinaryDescriptors(descName,
				featsBitstream,
				features,
				numFeatures);
	}
	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;
}

void SubtractBackgroundTask::execute(){

	Mat foreImage;
	absdiff(sourceImage,backgroundImage,foreImage); // compute the fore-ground image

	int pix_threshold = 8;
	bool ok1 = false, ok2 = false;
	long thr_c = pix_threshold * foreImage.rows;
	long thr_r = pix_threshold * foreImage.cols;

	// find horizontal margins
	// -----------------------------------------------------------------------
	bool no_obj_c = false;
	int min_c=0, max_c=foreImage.cols-1;
	Scalar sum_col1, sum_col2;

	while( ok1 == false || ok2 == false ){

		if( ok1 == false ){
			sum_col1 = sum(foreImage.col(min_c));
			if(sum_col1[0]>thr_c){
				ok1 = true;
			}
			else{
				min_c++;
			}
		}

		if( ok2 == false ){
			sum_col2 = sum(foreImage.col(max_c));
			if(sum_col2[0]>thr_c){
				ok2 = true;
			}
			else{
				max_c--;
			}
		}

		if( ok1 == false && ok2 == false && max_c < min_c){
			ok1 = true;
			ok2 = true;
			no_obj_c = true;
		}


	}


	// find vertical margins
	// -----------------------------------------------------------------------
	bool no_obj_r = false;
	int min_r=0, max_r=foreImage.rows-1;
	Scalar sum_row1, sum_row2;

	ok1 = false; ok2 = false;
	while( ok1 == false || ok2 == false ){

		if( ok1 == false ){
			sum_row1 = sum(foreImage.row(min_r));
			if(sum_row1[0]>thr_r){
				ok1 = true;
			}
			else{
				min_r++;
			}
		}

		if( ok2 == false ){
			sum_row2 = sum(foreImage.row(max_r));
			if(sum_row2[0]>thr_r){
				ok2 = true;
			}
			else{
				max_r--;
			}
		}

		if( ok1 == false && ok2 == false && max_r < min_r){
			ok1 = true;
			ok2 = true;
			no_obj_r = true;
		}


	}


	// if an object is present...
	// -----------------------------------------------------------------------
	if(no_obj_r==false && no_obj_c==false){

		// ... obtain the reduced image
		int margin = 60;

		min_c = max( 0 , min_c - margin );
		max_c = min( sourceImage.cols-1, max_c + margin );
		min_r = max( 0, min_r - margin );
		max_r = min( sourceImage.rows-1, max_r + margin );

		croppedImage = Mat(sourceImage, Range(min_r, max_r), Range(min_c, max_c)).clone();
	}
	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;

}

void ConvertColorspaceTask::execute(){
	if(mode==0){
		cvtColor(sourceImage, destImage, CV_BGR2GRAY);
	}
	else if(mode==1){
		cvtColor(sourceImage, destImage, CV_GRAY2BGR);
	}
	boost::mutex::scoped_lock lk(task_monitor);

	cout << "TM: task executed" << endl;
	completed = true;
}

SendMessageTask::~SendMessageTask(){
	//delete(msg_to_send);
}

SendMessageTask::SendMessageTask(Message *msg, serial_source radio_interface){
	msg_to_send = msg;
	type = SEND_MESSAGE_TASK;
	telosb = radio_interface;
	boost::mutex::scoped_lock lk(task_monitor);
	completed = false;
}

SendWiFiMessageTask::~SendWiFiMessageTask(){
	//delete(msg_to_send);
}

SendWiFiMessageTask::SendWiFiMessageTask(Message *msg){
	msg_to_send = msg;
	type = SEND_WIFI_MESSAGE_TASK;
	boost::mutex::scoped_lock lk(task_monitor);
	completed = false;
}

void SendWiFiMessageTask::execute(){

	// call the proper Connection.sendMsg method and then complete
	// make it synchrhonous so that we complete.

	Connection* dest = msg_to_send->getTcpConnection();
	dest->writeMsg(msg_to_send);
	boost::mutex::scoped_lock lk(task_monitor);
	completed = true;
}

void SendMessageTask::execute(){
	/*
	 *    packetize message according to interface and create queue of packets to transmit
	 *    while(!interrupted || no_more_packets){
	 *    	transmit a packet
	 *    }
	 *    completed = true;
	 */


	/****/
	// these should change based on the address of the message...
	int max_radio_packet_length = 122;
	int max_radio_payload_length = 114;
	int radio_header_length = 8;

	//this should be always the same
	int app_header_length = 10;
	/****/

	//this should be shared somewhare
	bool tx_interrupted = false;

	//retrieve tot number of packets to send
	vector<uchar> bitstream;
	msg_to_send->getBitStream(bitstream);
	cout << "SMT: bitstream size: " << bitstream.size() << endl;
	bool header_only_packet = false;


	int num_packets_to_send = ceil(double(bitstream.size())/ (double) (max_radio_payload_length-app_header_length) );
	cout << "SMT: total packets to send " << num_packets_to_send << endl;
	int remainingBytes = bitstream.size();

	if(remainingBytes == 0){
		header_only_packet = true;
		num_packets_to_send = 1;
	}

	int curFrameLength = max_radio_payload_length-app_header_length;
	int cur_packet = 0;
	int dataIdx = 0;
	while((remainingBytes>0 && !tx_interrupted) || header_only_packet){

		/****/
		//prepare packet (put it somewhere else, maybe in a class?)
		char serial_packet[max_radio_packet_length];

		//prepare header
		serial_packet[0] = 0x00;
		serial_packet[1] = 0x00;
		serial_packet[2] = 0x00;
		serial_packet[3] = 0x00;
		serial_packet[4] = 0x00;

		serial_packet[5] = max_radio_payload_length;
		serial_packet[6] = 0x00;
		serial_packet[7] = 0x47;

		//src address
		serial_packet[8] = (uchar)(msg_to_send->getSource()/256);
		serial_packet[9] = (uchar)(msg_to_send->getSource()-serial_packet[8]*256);

		//dest address
		serial_packet[10] = (uchar)(msg_to_send->getDestination()/256);
		serial_packet[11] = (uchar)(msg_to_send->getDestination()-serial_packet[10]*256);

		serial_packet[12]  = msg_to_send->getSeqNum();

		// use 2 bytes [9,10] for specifying the number of frames
		serial_packet[14] = (uchar)(num_packets_to_send/256);
		serial_packet[13]  = (uchar) (num_packets_to_send - serial_packet[14]*256);

		// set the frame number
		serial_packet[16] = (uchar)(cur_packet/256);
		serial_packet[15] = (uchar)(cur_packet - serial_packet[16]*256);

		// set the Message type
		serial_packet[17] = msg_to_send->getMessageType();

		// compute and set the length of the current frame
		if( remainingBytes < max_radio_payload_length-app_header_length){
			curFrameLength = remainingBytes;
			serial_packet[5] = app_header_length + curFrameLength;
		}
		/****/

		// fill the frame data
		for(int j=0; j<curFrameLength; j++){
			serial_packet[j+radio_header_length+app_header_length] = bitstream[dataIdx];
			dataIdx++;
		}
		cout << "SMT: packet ready" << endl;
		//send (telosb only)
		int ret = -1;
		int max_resend   = 7;
		int count_resend = 0;

		while(ret!=0 && count_resend<max_resend){
			ret = write_serial_packet(telosb, serial_packet, radio_header_length+app_header_length + curFrameLength);
			if (ret!=0 ){ //should be !=0 for certainty
				//cout << "ERROR SENDING FRAME TO RADIO SUBSYSTEM!" << endl;
				//cout << "RET " << ret << endl;
				count_resend++;
			}
			//usleep(30000);
		}
		if (msg_to_send->getDestination() == 0){
			cout << "SMT: packet sent to sink!" << endl;
		}
		else{
			cout << "SMT: packet sent to camera " << msg_to_send->getDestination() << "!" << endl;
		}
			
		//usleep(30000);
		remainingBytes-=curFrameLength;
		header_only_packet = false;
		cur_packet++;
	}
	boost::mutex::scoped_lock lk(task_monitor);
	cout << "TM: task executed" << endl;
	completed = true;
}

void ProbeAndSortLinkTask::execute(){
	offloading_mng->probeLinks();
	offloading_mng->sortCooperators();
	boost::mutex::scoped_lock lk(task_monitor);
	cout << "TM: task executed" << endl;
	completed = true;
}

void ComputeLoadsTask::execute(){
	myLoad = offloading_mng->computeLoads(img);
	boost::mutex::scoped_lock lk(task_monitor);
	cout << "TM: task executed" << endl;
	completed = true;
}

void TransmitLoadsTask::execute(){
	offloading_mng->transmitLoads();
	boost::mutex::scoped_lock lk(task_monitor);
	cout << "TM: task executed" << endl;
	completed = true;
}

/*void MergeSubSlicesTask::execute(){
	processing_mng->mergeSubSlices(subslices_iteration_,subsliceList_);
	boost::mutex::scoped_lock lk(task_monitor);
	cout << "TM: task executed" << endl;
	completed = true;
}*/
SIMULATIONTask::~SIMULATIONTask(){
	//delete(msg_to_send);
}

SIMULATIONTask::SIMULATIONTask(Message *msg){
	msg_to_send = msg;
	type = SIMULATION_TASK;
	boost::mutex::scoped_lock lk(task_monitor);
	completed = false;
}
void SIMULATIONTask::execute(){
	
	sleep(2);
	NodeManager* node_mng;
	node_mng->notify_msg(msg_to_send);
	boost::mutex::scoped_lock lk(task_monitor);
	completed = true;

}
