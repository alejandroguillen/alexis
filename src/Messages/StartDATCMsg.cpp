/*
 * StartDATCMsg.cpp
 *
 *  Created on: Jul 8, 2014
 *      Author: Ale
 */
#include"StartDATCMsg.h"

StartDATCMsg::~StartDATCMsg(){
	ASN_STRUCT_FREE(asn_DEF_StartDATCMessage, internal_msg);
	//free(internal_msg);
	//internal_msg = NULL;
}

StartDATCMsg::StartDATCMsg(int fps, DetectorTypes_t det, double det_thr, DescriptorTypes_t desc, int desc_length,
		int max_feat, bool rotation_invariant, CodingChoices_t coding, bool transfer_kpt, bool transfer_scale,
        bool transfer_orientation, int num_feat_per_block, int num_cooperators){
	msg_type = START_DATC_MESSAGE;
	internal_msg = (StartDATCMessage_t*) calloc(1, sizeof(*internal_msg));
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
    internal_msg->numCooperators = num_cooperators;
	//xer_fprint(stdout, &asn_DEF_StartATCMessage, internal_msg);
}
StartDATCMsg::StartDATCMsg(StartDATCMessage_t* internal_message){
	msg_type = START_DATC_MESSAGE;
	internal_msg = internal_message;
}

void StartDATCMsg::setFramesPerSecond(int fps){
	internal_msg->framesPerSecond = fps;
}

void StartDATCMsg::setDetectorThreshold(double det_thr){
	internal_msg->detectorThreshold = det_thr;
}

void StartDATCMsg::setDetectorType(DetectorTypes_t det){
	internal_msg->detectorType = det;
}

void StartDATCMsg::setDescriptorLength(int desc_length){
	internal_msg->descriptorLength = desc_length;
}

void StartDATCMsg::setDescriptorType(DescriptorTypes_t desc){
	internal_msg->descriptorType = desc;
}

void StartDATCMsg::setCoding(CodingChoices_t coding){
	internal_msg->coding = coding;
}

void StartDATCMsg::setMaxNumFeat(int max_feat){
	internal_msg->maxNumberOfFeatures = max_feat;
}

void StartDATCMsg::setTransferKpt(bool transfer_kpt){
	internal_msg->transferCoordinates = transfer_kpt;
}

void StartDATCMsg::setTransferOrientation(bool transfer_orientation){
	internal_msg->transferOrientation = transfer_orientation;
}

void StartDATCMsg::setTransferScale(bool transfer_scale){
	internal_msg->transferScale = transfer_scale;
}

void StartDATCMsg::setNumFeatPerBlocks(int n){
	internal_msg->numFeaturesPerBlock = n;
}

void StartDATCMsg::setNumCooperators(int n){
	internal_msg->numCooperators = n;
}

int StartDATCMsg::getDescriptorLength(){
	return internal_msg->descriptorLength;
}

int StartDATCMsg::getFramesPerSecond(){
	return internal_msg->framesPerSecond;
}

int StartDATCMsg::getMaxNumFeat(){
	return internal_msg->maxNumberOfFeatures;
}

CodingChoices_t StartDATCMsg::getCoding(){
	return internal_msg->coding;
}

double StartDATCMsg::getDetectorThreshold(){
	return internal_msg->detectorThreshold;
}

int StartDATCMsg::getNumFeatPerBlock(){
	return internal_msg->numFeaturesPerBlock;
}


DescriptorTypes_t StartDATCMsg::getDescriptorType(){
	return internal_msg->descriptorType;
}

DetectorTypes_t StartDATCMsg::getDetectorType(){
	return internal_msg->detectorType;
}

bool StartDATCMsg::getTransferKpt(){
	return internal_msg->transferCoordinates;
}

bool StartDATCMsg::getTransferOrientation(){
	return internal_msg->transferOrientation;
}

bool StartDATCMsg::getTransferScale(){
	return internal_msg->transferScale;
}

int StartDATCMsg::getNumCooperators(){
	return internal_msg->numCooperators;
}

int StartDATCMsg::getBitStream(vector<uchar>& bitstream){
	uchar buf[MAX_START_DATC_MESSAGE_SIZE];         /* Temporary buffer - 1K*/
	asn_enc_rval_t ec;

	ec = uper_encode_to_buffer(&asn_DEF_StartDATCMessage,internal_msg, buf, MAX_START_DATC_MESSAGE_SIZE);
	for(int i=0;i<ceil(double(ec.encoded)/8);i++){
		bitstream.push_back(buf[i]);
	}
	return ec.encoded;
}
