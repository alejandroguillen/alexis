/*
 * StopMsg.h
 *
 *  Created on: 24/lug/2014
 *      Author: greeneyes
 */

#ifndef STOPMSG_H_
#define STOPMSG_H_

#include "Message.h"

class StopMsg : public Message{
private:

public:
	StopMsg();
	int getBitStream(vector<uchar>& bitstream);
};


#endif /* STOPMSG_H_ */
