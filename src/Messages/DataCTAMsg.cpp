/*
 * DataCTAMsg.cpp
 *
 *  Created on: Jul 9, 2014
 *      Author: Ale
 */
#include "DataCTAMsg.h"

DataCTAMsg::~DataCTAMsg(){
	//ASN_STRUCT_FREE(asn_DEF_DataCTAMessage, internal_msg);
	//free(internal_msg);
	//internal_msg = NULL;
}

DataCTAMsg::DataCTAMsg(int frameID, int sliceNumber, Coordinate_t topLeft, int dataSize,
		double encTime, double txTime, vector<uchar>& data){
	msg_type = DATA_CTA_MESSAGE;
	internal_msg = (DataCTAMessage_t*) calloc(1, sizeof(*internal_msg));
	assert(internal_msg);
	internal_msg->frameID = frameID;
	internal_msg->sliceNumber = sliceNumber;
	internal_msg->topLeft = topLeft;
	internal_msg->dataSize = dataSize;
	internal_msg->encTime = encTime;
	internal_msg->txTime = txTime;
	OCTET_STRING_t data_str;

	data_str.size = data.size();
	data_str.buf = static_cast<uint8_t*>(&data[0]);
	internal_msg->data = data_str;
}

DataCTAMsg::DataCTAMsg(DataCTAMessage_t *internal_message){
	msg_type = DATA_CTA_MESSAGE;
	internal_msg = internal_message;
}

int DataCTAMsg::getFrameId(){
	return internal_msg->frameID;
}
int DataCTAMsg::getSliceNumber(){
	return internal_msg->sliceNumber;
}
Coordinate_t DataCTAMsg::getTopLeft(){
	return internal_msg->topLeft;
}
int DataCTAMsg::getDataSize(){
	return internal_msg->dataSize;
}
double DataCTAMsg::getEncodingTime(){
	return internal_msg->encTime;
}
double DataCTAMsg::getTxTime(){
	return internal_msg->txTime;
}
void DataCTAMsg::setTxTime(double tx_time){
	internal_msg->txTime = tx_time;
}

OCTET_STRING_t DataCTAMsg::getData(){
	return internal_msg->data;
}

int DataCTAMsg::getBitStream(vector<uchar>& bitstream){
	uchar buf[MAX_DATA_CTA_MESSAGE_SIZE];         /* Temporary buffer - 10K*/
	asn_enc_rval_t ec;

	ec = uper_encode_to_buffer(&asn_DEF_DataCTAMessage,internal_msg, buf, MAX_DATA_CTA_MESSAGE_SIZE);
	for(int i=0;i<ceil(double(ec.encoded)/8);i++){
		bitstream.push_back(buf[i]);
	}
	return ec.encoded;
}

