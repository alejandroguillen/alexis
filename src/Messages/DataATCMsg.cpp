/*
 * DataATCMsg.cpp
 *
 *  Created on: Jul 11, 2014
 *      Author: Ale
 */


/*
 * DataCTAMsg.cpp
 *
 *  Created on: Jul 9, 2014
 *      Author: Ale
 */
#include "DataATCMsg.h"

DataATCMsg::~DataATCMsg(){
	//ASN_STRUCT_FREE(asn_DEF_DataATCMessage, internal_msg);
	//free(internal_msg);
	//internal_msg = NULL;
}

DataATCMsg::DataATCMsg(int frameID, int blockNumber, int numBlocks, double detTime,
		double descTime, double kptsEncTime, double featEncTime, double txTime, int numFeat, int  numKpts,
		vector<uchar>& features_data, vector<uchar>& keypoints_data){
	msg_type = DATA_ATC_MESSAGE;
	internal_msg = (DataATCMessage_t*) calloc(1, sizeof(*internal_msg));
	assert(internal_msg);
	internal_msg->frameID = frameID;
	internal_msg->blockNumber = blockNumber;
	internal_msg->numBlocks = numBlocks;
	internal_msg->detTime = detTime;
	internal_msg->descTime = descTime;
	internal_msg->kencTime = kptsEncTime;
	internal_msg->fencTime = featEncTime;
	internal_msg->txTime = txTime;
	internal_msg->numFeat = numFeat;
	internal_msg->numKpts = numKpts;
	OCTET_STRING_t ft_data;
	OCTET_STRING_t kp_data;

	ft_data.size = features_data.size();
	ft_data.buf = static_cast<uint8_t*>(&features_data[0]);
	internal_msg->featuresData = ft_data;

	//todo: modify this... should probably check if the pointer is != from null
	if(keypoints_data.size() != 0){
		kp_data.size = keypoints_data.size();
		kp_data.buf = static_cast<uint8_t*>(&keypoints_data[0]);
		internal_msg->kptsData = kp_data;
	}
}

DataATCMsg::DataATCMsg(DataATCMessage_t *internal_message){
	msg_type = DATA_ATC_MESSAGE;
	internal_msg = internal_message;
}

int DataATCMsg::getNumFeat(){
	return internal_msg->numFeat;
}

int DataATCMsg::getNumKpts(){
	return internal_msg->numKpts;
}

int DataATCMsg::getFrameId(){
	return internal_msg->frameID;
}
int DataATCMsg::getBlockNumber(){
	return internal_msg->blockNumber;
}

int DataATCMsg::getNumBlocks(){
	return internal_msg->numBlocks;
}

double DataATCMsg::getDetTime(){
	return internal_msg->detTime;
}

double DataATCMsg::getDescTime(){
	return internal_msg->descTime;
}

double DataATCMsg::getTxTime(){
	return internal_msg->txTime;
}
void DataATCMsg::setTxTime(double tx_time){
	internal_msg->txTime = tx_time;
}

double DataATCMsg::getFeatEncodingTime(){
	return internal_msg->fencTime;
}

double DataATCMsg::getKptsEncodingTime(){
	return internal_msg->kencTime;
}

OCTET_STRING_t DataATCMsg::getFeaturesData(){
	return internal_msg->featuresData;
}

OCTET_STRING_t DataATCMsg::getKeypointsData(){
	return internal_msg->kptsData;
}

int DataATCMsg::getBitStream(vector<uchar>& bitstream){
	uchar buf[MAX_DATA_ATC_MESSAGE_SIZE];         /* Temporary buffer - 10K*/
	asn_enc_rval_t ec;

	ec = uper_encode_to_buffer(&asn_DEF_DataATCMessage,internal_msg, buf, MAX_DATA_ATC_MESSAGE_SIZE);
	for(int i=0;i<ceil(double(ec.encoded)/8);i++){
		bitstream.push_back(buf[i]);
	}
	return ec.encoded;
}


