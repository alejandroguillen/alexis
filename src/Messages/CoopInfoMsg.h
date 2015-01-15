/*
 * CoopInfoMsg.h
 *
 *  Created on: 18/set/2014
 *      Author: greeneyes
 */

#ifndef COOPINFOMSG_H_
#define COOPINFOMSG_H_
#include "Message.h"
#include <string>

class CoopInfoMsg : public Message{
private:
	CooperatorInfo_t* internal_msg;

public:
	~CoopInfoMsg();
	CoopInfoMsg(std::string ip_address, int port, CoopStatus_t status);
	CoopInfoMsg(CooperatorInfo_t* internal_message);
	int getBitStream(vector<uchar>& bitstream);

	int getPort();
	std::string getIpAddress();
	CoopStatus_t getStatus();

	/*int getFrameId();
	int getBlockNumber();
	OCTET_STRING_t getFeaturesData();
	OCTET_STRING_t getKeypointsData();*/ //may return directly the opencv Mat
};


#endif /* COOPINFOMSG_H_ */
