/*
 * StartATCMsg.cpp
 *
 *  Created on: Jul 8, 2014
 *      Author: Ale
 */
#include"StartATCMsg.h"

StartATCMsg::~StartATCMsg(){
	ASN_STRUCT_FREE(asn_DEF_StartATCMessage, internal_msg);
	//free(internal_msg);
	//internal_msg = NULL;
}

StartATCMsg::StartATCMsg(int fps, DetectorTypes_t det, double det_thr, DescriptorTypes_t desc, int desc_length,
		int max_feat, bool rotation_invariant, CodingChoices_t coding, bool transfer_kpt, bool transfer_scale,
        bool transfer_orientation, int num_feat_per_block){
	msg_type = START_ATC_MESSAGE;
	internal_msg = (StartATCMessage_t*) calloc(1, sizeof(*internal_msg));
	assert(internal_msg);
	internal_msg->framesPerSecond = fps;
	internal_msg->detectorType = det;
	internal_msg->detectorThreshold = det_thr;
	internal_msg->descriptorType = desc;
	internal_msg->descriptorLength = desc_length;
	internal_msg->maxNumberOfFeatures = max_feat;
	internal_msg->rotationInvariant = rotation_invariant;
	internal_msg->coding = coding;
	internal_msg->transferCoordinates = transfer_kpt;
	internal_msg->transferScale = transfer_scale;
	internal_msg->transferOrientation = transfer_orientation;
    internal_msg->numFeaturesPerBlock = num_feat_per_block;
	//xer_fprint(stdout, &asn_DEF_StartATCMessage, internal_msg);
}
StartATCMsg::StartATCMsg(StartATCMessage_t* internal_message){
	msg_type = START_ATC_MESSAGE;
	internal_msg = internal_message;
}

void StartATCMsg::setFramesPerSecond(int fps){
	internal_msg->framesPerSecond = fps;
}

void StartATCMsg::setDetectorThreshold(double det_thr){
	internal_msg->detectorThreshold = det_thr;
}

void StartATCMsg::setDetectorType(DetectorTypes_t det){
	internal_msg->detectorType = det;
}

void StartATCMsg::setDescriptorLength(int desc_length){
	internal_msg->descriptorLength = desc_length;
}

void StartATCMsg::setDescriptorType(DescriptorTypes_t desc){
	internal_msg->descriptorType = desc;
}

void StartATCMsg::setCoding(CodingChoices_t coding){
	internal_msg->coding = coding;
}

void StartATCMsg::setMaxNumFeat(int max_feat){
	internal_msg->maxNumberOfFeatures = max_feat;
}

void StartATCMsg::setTransferKpt(bool transfer_kpt){
	internal_msg->transferCoordinates = transfer_kpt;
}

void StartATCMsg::setTransferOrientation(bool transfer_orientation){
	internal_msg->transferOrientation = transfer_orientation;
}

void StartATCMsg::setTransferScale(bool transfer_scale){
	internal_msg->transferScale = transfer_scale;
}

void StartATCMsg::setNumFeatPerBlock(int n){
	internal_msg->numFeaturesPerBlock = n;
}

int StartATCMsg::getDescriptorLength(){
	return internal_msg->descriptorLength;
}

int StartATCMsg::getFramesPerSecond(){
	return internal_msg->framesPerSecond;
}

int StartATCMsg::getMaxNumFeat(){
	return internal_msg->maxNumberOfFeatures;
}

CodingChoices_t StartATCMsg::getCoding(){
	return internal_msg->coding;
}

double StartATCMsg::getDetectorThreshold(){
	return internal_msg->detectorThreshold;
}

int StartATCMsg::getNumFeatPerBlock(){
	return internal_msg->numFeaturesPerBlock;
}


DescriptorTypes_t StartATCMsg::getDescriptorType(){
	return internal_msg->descriptorType;
}

DetectorTypes_t StartATCMsg::getDetectorType(){
	return internal_msg->detectorType;
}

bool StartATCMsg::getTransferKpt(){
	return internal_msg->transferCoordinates;
}

bool StartATCMsg::getTransferOrientation(){
	return internal_msg->transferOrientation;
}

bool StartATCMsg::getTransferScale(){
	return internal_msg->transferScale;
}

int StartATCMsg::getBitStream(vector<uchar>& bitstream){
	uchar buf[MAX_START_ATC_MESSAGE_SIZE];         /* Temporary buffer - 1K*/
	asn_enc_rval_t ec;

	ec = uper_encode_to_buffer(&asn_DEF_StartATCMessage,internal_msg, buf, MAX_START_ATC_MESSAGE_SIZE);
	for(int i=0;i<ceil(double(ec.encoded)/8);i++){
		bitstream.push_back(buf[i]);
	}
	return ec.encoded;
}
