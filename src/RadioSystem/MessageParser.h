/*
 * MessageParser.h
 *
 *  Created on: 25/set/2014
 *      Author: greeneyes
 */

#ifndef MESSAGEPARSER_H_
#define MESSAGEPARSER_H_

#include "Messages/Message.h"
#include "Messages/Header.h"

class MessageParser{
public:
	Header* parseHeader(uchar* bitstream);
	Message* parseMessage(Header*h, uchar* bitstream, Connection* cn);
	Message* parseMessage(Header* h, uchar* bitstream);

};



#endif /* MESSAGEPARSER_H_ */
