/*
 * DataCTAMsg.h
 *
 *  Created on: Jul 9, 2014
 *      Author: Ale
 */

#ifndef DATACTAMSG_H_
#define DATACTAMSG_H_
#include <vector>
#include "Message.h"

using namespace std;

class DataCTAMsg : public Message{
private:
	//DataCTAMessage_t* internal_msg;

public:
	DataCTAMessage_t* internal_msg;
	~DataCTAMsg();
	//DataCTAMessage_t* internal_msg;
	DataCTAMsg(int frameID, int sliceNumber, Coordinate_t topLeft, int dataSize,
			double encTime, double txTime, vector<uchar>& data);
	DataCTAMsg(DataCTAMessage_t* internal_message);
	int getBitStream(vector<uchar>& bitstream);

	int getFrameId();
	int getSliceNumber();
	Coordinate_t getTopLeft();
	int getDataSize();
	double getEncodingTime();
	double getTxTime();
	void setTxTime(double tx_time);
	OCTET_STRING_t getData(); //may return directly the opencv Mat
};


#endif /* DATACTAMSG_H_ */
