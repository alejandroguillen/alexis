/*
 * AddCameraMsg.cpp
 *
 *  Created on: 11/01/2015
 *      Author: Alejandro Guillen
 */
#include "AddCameraMsg.h"

AddCameraMsg::~AddCameraMsg(){
	ASN_STRUCT_FREE(asn_DEF_AddCameraMessage, internal_msg);
	//free(internal_msg);
	//internal_msg = NULL;
}

AddCameraMsg::AddCameraMsg(int cameraid){
	msg_type = ADD_CAMERA_MESSAGE;
	internal_msg = (AddCameraMessage_t*) calloc(1, sizeof(*internal_msg));
	assert(internal_msg);
	internal_msg->cameraid = cameraid;
}

AddCameraMsg::AddCameraMsg(AddCameraMessage_t *internal_message){
	msg_type = ADD_CAMERA_MESSAGE;
	internal_msg = internal_message;
}

int AddCameraMsg::getBitStream(vector<uchar>& bitstream){
	uchar buf[MAX_ADD_CAMERA_MESSAGE_SIZE];         /* Temporary buffer - 10K*/
	asn_enc_rval_t ec;

	ec = uper_encode_to_buffer(&asn_DEF_AddCameraMessage,internal_msg, buf, MAX_ADD_CAMERA_MESSAGE_SIZE);
	for(int i=0;i<ceil(double(ec.encoded)/8);i++){
		bitstream.push_back(buf[i]);
	}
	return ec.encoded;
}

void AddCameraMsg::setCameraID(int cameraid) {
	internal_msg->cameraid=cameraid;
}

int AddCameraMsg::getCameraID() {
	return internal_msg->cameraid;
}


