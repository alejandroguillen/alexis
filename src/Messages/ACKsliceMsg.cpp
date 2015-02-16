/*
 * ACKsliceMsg.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: jordi
 */

#include "ACKsliceMsg.h"

ACKsliceMsg::~ACKsliceMsg() {
	ASN_STRUCT_FREE(asn_DEF_ACKsliceMessage, internal_msg);
}

ACKsliceMsg::ACKsliceMsg(int frameID) {
	msg_type = ACK_SLICE_MESSAGE;
	internal_msg = (ACKsliceMessage_t*) calloc(1, sizeof(*internal_msg));
	assert(internal_msg);
	internal_msg->frameID=frameID;
}

ACKsliceMsg::ACKsliceMsg(ACKsliceMessage_t* internal_message) {
	msg_type = ACK_SLICE_MESSAGE;
	internal_msg = internal_message;
}

int ACKsliceMsg::getBitStream(vector<uchar>& bitstream) {
	uchar buf[MAX_ACK_SLICE_MESSAGE_SIZE];         /* Temporary buffer - 10K*/
	asn_enc_rval_t ec;

	ec = uper_encode_to_buffer(&asn_DEF_ACKsliceMessage,internal_msg, buf, MAX_ACK_SLICE_MESSAGE_SIZE);
	for(int i=0;i<ceil(double(ec.encoded)/8);i++){
		bitstream.push_back(buf[i]);
	}
	return ec.encoded;
}

void ACKsliceMsg::setFrameID(int id) {
	internal_msg->frameID=id;
}

int ACKsliceMsg::getFrameID() {
	return internal_msg->frameID;
}
