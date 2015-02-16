#ifndef MESSAGE_H
#define MESSAGE_H


#include <iostream>
#include <vector>
#include <math.h>

#include <opencv2/core/core.hpp>
#include "ASN.1/StartATCMessage.h"
#include "ASN.1/StartCTAMessage.h"
#include "ASN.1/StartDATCMessage.h"
#include "ASN.1/DataCTAMessage.h"
#include "ASN.1/DataATCMessage.h"
#include "ASN.1/CooperatorInfo.h"
#include "ASN.1/ACKsliceMessage.h"
#include "ASN.1/AddCameraMessage.h"
#include <boost/asio.hpp>


using namespace std;
using boost::asio::ip::tcp;

#define MAX_COOP_INFO_MESSAGE_SIZE 1024
#define MAX_START_CTA_MESSAGE_SIZE 1024
#define MAX_START_ATC_MESSAGE_SIZE 1024
#define MAX_START_DATC_MESSAGE_SIZE 1024
#define MAX_DATA_CTA_MESSAGE_SIZE 1024000 //1MB... a LOT!!
#define MAX_DATA_ATC_MESSAGE_SIZE 1024000
#define MAX_ACK_SLICE_MESSAGE_SIZE 1024
#define MAX_ADD_CAMERA_MESSAGE_SIZE 1024

enum MessageType{
	START_CTA_MESSAGE,
	START_ATC_MESSAGE,
	START_DATC_MESSAGE,
	DATA_CTA_MESSAGE,
	DATA_ATC_MESSAGE,
	STOP_MESSAGE,
	COOP_INFO_MESSAGE,
	ACK_SLICE_MESSAGE,
	ADD_CAMERA_MESSAGE
};

class Connection;

class Message{
private:

	//todo Address class

	//TinyOS/Telosb Address
	int src_addr;
	int dst_addr;

	//WiFi TCP socket
	Connection* connection;



public:
	virtual ~Message(){};
	MessageType msg_type;
    uint8_t seq_num;


	MessageType getMessageType(){
		return msg_type;
	}

	uint8_t getSeqNum(){
		return seq_num;
	}

	void setSeqNum(uint8_t seqnum){
		seq_num = seqnum;
	}

	void setSource(int src){
		src_addr = src;
	}

	int getSource(){
		return src_addr;
	}

	void setDestination(int dst){
		dst_addr = dst;
	}

	int getDestination(){
		return dst_addr;
	}

	Connection* getTcpConnection(){
		return connection;
	}

	void setTcpConnection(Connection* cn){
		connection = cn;
	}


	virtual int getBitStream(vector<uchar>& bitstream) = 0;


};


#endif
