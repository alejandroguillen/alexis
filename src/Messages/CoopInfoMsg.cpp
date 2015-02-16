/*
 * CoopInfoMsg.cpp
 *
 *  Created on: 18/set/2014
 *      Author: greeneyes
 */




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
#include "CoopInfoMsg.h"

CoopInfoMsg::~CoopInfoMsg(){
	ASN_STRUCT_FREE(asn_DEF_CooperatorInfo, internal_msg);
	//free(internal_msg);
	//internal_msg = NULL;
}

CoopInfoMsg::CoopInfoMsg(std::string ip_address, int port, CoopStatus_t status){
	msg_type = COOP_INFO_MESSAGE;
	internal_msg = (CooperatorInfo_t*) calloc(1, sizeof(*internal_msg));
	assert(internal_msg);
	OCTET_STRING_t address;
	address.size = ip_address.size();
	address.buf = reinterpret_cast<uint8_t*>(&ip_address[0]);
	internal_msg->ipAddress = address;
	internal_msg->port = port;
	internal_msg->status = status;
}

CoopInfoMsg::CoopInfoMsg(CooperatorInfo_t *internal_message){
	msg_type = COOP_INFO_MESSAGE;
	internal_msg = internal_message;
}

int CoopInfoMsg::getPort(){
	return internal_msg->port;
}

CoopStatus_t CoopInfoMsg::getStatus(){
	return internal_msg->status;
}

std::string CoopInfoMsg::getIpAddress(){
	std::string ip_addr(internal_msg->ipAddress.buf, internal_msg->ipAddress.buf + internal_msg->ipAddress.size);
	return ip_addr;
}

int CoopInfoMsg::getBitStream(vector<uchar>& bitstream){
	uchar buf[MAX_COOP_INFO_MESSAGE_SIZE];         /* Temporary buffer - 10K*/
	asn_enc_rval_t ec;

	ec = uper_encode_to_buffer(&asn_DEF_CooperatorInfo,internal_msg, buf, MAX_COOP_INFO_MESSAGE_SIZE);
	for(int i=0;i<ceil(double(ec.encoded)/8);i++){
		bitstream.push_back(buf[i]);
	}
	return ec.encoded;
}


