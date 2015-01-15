/*
 * StartCTAMsg.cpp
 *
 *  Created on: Jul 8, 2014
 *      Author: Ale
 */
#include "StartCTAMsg.h"

//START CTA
using namespace std;
ostream& operator <<(ostream& os, StartCTAMsg mex){

    os << "fps: " << mex.getFramesPerSecond() << endl;
    os << "quality factor: " << mex.getQualityFactor() << endl;
    os << "image width: " << mex.getFrameWidth() << endl;
    os << "image height: " << mex.getFrameHeight() << endl;

    return os;
}

StartCTAMsg::~StartCTAMsg(){
	ASN_STRUCT_FREE(asn_DEF_StartCTAMessage, internal_msg);
	//free(internal_msg);
	//internal_msg = NULL;
}

StartCTAMsg::StartCTAMsg(int fps, int qf, int h, int w, int num_slices){
	msg_type = START_CTA_MESSAGE;
	internal_msg = (StartCTAMessage_t*) calloc(1, sizeof(*internal_msg));
	assert(internal_msg);
	internal_msg->framesPerSecond = fps;
	internal_msg->qualityFactor = qf;
	internal_msg->frameHeight = h;
	internal_msg->frameWidth = w;
    internal_msg->numSlices = num_slices;
	//xer_fprint(stdout, &asn_DEF_StartCTAMessage, internal_msg);
}
StartCTAMsg::StartCTAMsg(StartCTAMessage_t* internal_message){
	msg_type = START_CTA_MESSAGE;
	internal_msg = internal_message;
}

void StartCTAMsg::setFramesPerSecond(int fps){
	internal_msg->framesPerSecond = fps;
}
void StartCTAMsg::setQualityFactor(int qf){
	internal_msg->qualityFactor = qf;
}
void StartCTAMsg::setFrameWidth(int w){
	internal_msg->frameWidth = w;
}
void StartCTAMsg::setFrameHeight(int h){
	internal_msg->frameHeight = h;
}
void StartCTAMsg::setNumSlices(int n){
	internal_msg->numSlices = n;
}

int StartCTAMsg::getFramesPerSecond(){
	return internal_msg->framesPerSecond;
}
int StartCTAMsg::getQualityFactor(){
	return internal_msg->qualityFactor;
}
int StartCTAMsg::getFrameHeight(){
	return internal_msg->frameHeight;
}
int StartCTAMsg::getFrameWidth(){
	return internal_msg->frameWidth;
}
int StartCTAMsg::getNumSlices(){
	return internal_msg->numSlices;
}

int StartCTAMsg::getBitStream(vector<uchar>& bitstream){
	uchar buf[MAX_START_CTA_MESSAGE_SIZE];         /* Temporary buffer - 1K*/
	asn_enc_rval_t ec;

	ec = uper_encode_to_buffer(&asn_DEF_StartCTAMessage,internal_msg, buf, MAX_START_ATC_MESSAGE_SIZE);
	for(int i=0;i<ceil(double(ec.encoded)/8);i++){
		bitstream.push_back(buf[i]);
	}


	return ec.encoded;
}



