/*
 * ACKsliceMsg.h
 *
 *  Created on: Oct 22, 2014
 *      Author: jordi
 */

#ifndef ACKSLICEMSG_H_
#define ACKSLICEMSG_H_

#include "Message.h"

class ACKsliceMsg : public Message{
private:
	ACKsliceMessage_t* internal_msg;

public:
	~ACKsliceMsg();
	ACKsliceMsg(int frameID);
	ACKsliceMsg(ACKsliceMessage_t* internal_message);
	int getBitStream(vector<uchar>& bitstream);

	void setFrameID(int id);
	int getFrameID();
};

#endif /* ACKSLICEMSG_H_ */
