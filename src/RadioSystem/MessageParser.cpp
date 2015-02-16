/*
 * MessageParser.cpp
 *
 *  Created on: 25/set/2014
 *      Author: greeneyes
 */

#include "MessageParser.h"
#include "Messages/StartCTAMsg.h"
#include "Messages/StartATCMsg.h"
#include "Messages/StartDATCMsg.h"
#include "Messages/DataATCMsg.h"
#include "Messages/DataCTAMsg.h"
#include "Messages/CoopInfoMsg.h"
#include "Messages/StopMsg.h"
#include "Messages/Header.h"
#include "Messages/ACKsliceMsg.h"
#include "Messages/AddCameraMsg.h"

Header* MessageParser::parseHeader(uchar* bitstream){
	vector<unsigned char> vec;

	for (int i = 0; i < HEADER_SIZE; i++){
		vec.push_back(bitstream[i]);
	}

	Header* h = new Header(vec);
	return h;
}
Message* MessageParser::parseMessage(Header* h, uchar* bitstream){
	return parseMessage(h,bitstream,0);
}

Message* MessageParser::parseMessage(Header* h, uchar* bitstream, Connection* cn){
	Message* msg = 0;


	switch(h->getMsgT()){

	case START_CTA_MESSAGE:
	{
		cout << "Message is START_CTA" << endl;

		char buf[MAX_START_CTA_MESSAGE_SIZE];

		cout << "Deserializing start cta message" << endl;
		int bitstream_size = h->getPayloadSize();
		cout << "Bitstream size is " << bitstream_size << endl;

		//copy the bitstream (MAYBE REMOVED?)
		for(int i=0;i<bitstream_size;i++){
			buf[i] = bitstream[i];
		}

		StartCTAMessage_t* internal_message = (StartCTAMessage_t*) calloc(1, sizeof(*internal_message));
		asn_dec_rval_t rval;

		rval = uper_decode_complete(0, &asn_DEF_StartCTAMessage,(void **)&internal_message, buf, MAX_START_CTA_MESSAGE_SIZE);
		msg = new StartCTAMsg(internal_message);
		msg->setSource(h->getSrcAddr());
		msg->setDestination(h->getDstAddr());
		msg->setTcpConnection(cn);



		if(rval.code != RC_OK) {
			fprintf(stderr,
					"Broken message encoding at byte %ld\n",
					(long)rval.consumed);
			//exit(65); /* better, EX_DATAERR */
			msg = NULL;
		} else {
			//fprintf(stdout,"Printing msg as XML...\n");
			//xer_fprint(stdout, &asn_DEF_StartCTAMessage, internal_message);
		}

		break;
	}

	case START_ATC_MESSAGE:
	{
		cout << "Message is START_ATC" << endl;

		char buf[MAX_START_ATC_MESSAGE_SIZE];

		cout << "Deserializing start atc message" << endl;
		int bitstream_size = h->getPayloadSize();
		cout << "Bitstream size is " << bitstream_size << endl;

		//copy the bitstream (MAYBE REMOVED)
		for(int i=0;i<bitstream_size;i++){
			buf[i] = bitstream[i];
		}

		StartATCMessage_t* internal_message = (StartATCMessage_t*) calloc(1, sizeof(*internal_message));
		asn_dec_rval_t rval;
		rval = uper_decode_complete(0, &asn_DEF_StartATCMessage,(void **)&internal_message, buf, MAX_START_ATC_MESSAGE_SIZE);
		msg = new StartATCMsg(internal_message);
		msg->setSource(h->getSrcAddr());
		msg->setDestination(h->getDstAddr());
		msg->setTcpConnection(cn);

		if(rval.code != RC_OK) {
			fprintf(stderr,
					"Broken message encoding at byte %ld\n",
					(long)rval.consumed);
			//exit(65); /* better, EX_DATAERR */
			msg = NULL;
		} else {
			fprintf(stdout,"Printing msg as XML...\n");
			//xer_fprint(stdout, &asn_DEF_StartATCMessage, internal_message);
		}

		break;
	}

	case START_DATC_MESSAGE:
	{
		cout << "Message is START_DATC" << endl;

		char buf[MAX_START_DATC_MESSAGE_SIZE];

		cout << "Deserializing start datc message" << endl;
		int bitstream_size = h->getPayloadSize();
		cout << "Bitstream size is " << bitstream_size << endl;

		//copy the bitstream (MAYBE REMOVED)
		for(int i=0;i<bitstream_size;i++){
			buf[i] = bitstream[i];
		}

		StartDATCMessage_t* internal_message = (StartDATCMessage_t*) calloc(1, sizeof(*internal_message));
		asn_dec_rval_t rval;
		rval = uper_decode_complete(0, &asn_DEF_StartDATCMessage,(void **)&internal_message, buf, MAX_START_DATC_MESSAGE_SIZE);
		msg = new StartDATCMsg(internal_message);
		msg->setSource(h->getSrcAddr());
		msg->setDestination(h->getDstAddr());
		msg->setTcpConnection(cn);

		if(rval.code != RC_OK) {
			fprintf(stderr,
					"Broken message encoding at byte %ld\n",
					(long)rval.consumed);
			//exit(65); /* better, EX_DATAERR */
			msg = NULL;
		} else {
			//fprintf(stdout,"Printing msg as XML...\n");
			//xer_fprint(stdout, &asn_DEF_StartDATCMessage, internal_message);
		}

		break;
	}

	case DATA_CTA_MESSAGE:
	{
		cout << "Message is DATA_CTA" << endl;

		char buf[MAX_DATA_CTA_MESSAGE_SIZE];

		cout << "Deserializing data cta message" << endl;
		int bitstream_size = h->getPayloadSize();
		cout << "Bitstream size is " << bitstream_size << endl;

		//copy the bitstream (MAYBE REMOVED)
		for(int i=0;i<bitstream_size;i++){
			buf[i] = bitstream[i];
		}

		DataCTAMessage_t* internal_message = (DataCTAMessage_t*) calloc(1, sizeof(*internal_message));
		asn_dec_rval_t rval;
		rval = uper_decode_complete(0, &asn_DEF_DataCTAMessage,(void **)&internal_message, buf, MAX_DATA_CTA_MESSAGE_SIZE);
		msg = new DataCTAMsg(internal_message);
		msg->setSource(h->getSrcAddr());
		msg->setDestination(h->getDstAddr());
		msg->setTcpConnection(cn);

		if(rval.code != RC_OK) {
			fprintf(stderr,
					"Broken message encoding at byte %ld\n",
					(long)rval.consumed);
			//exit(65); /* better, EX_DATAERR */
			msg = NULL;
		} else {
			//fprintf(stdout,"Printing msg as XML...\n");
			//xer_fprint(stdout, &asn_DEF_DataCTAMessage, internal_message);
		}

		break;
	}

	case DATA_ATC_MESSAGE:
	{
		cout << "Message is DATA_ATC" << endl;

		char buf[MAX_DATA_ATC_MESSAGE_SIZE];

		cout << "Deserializing data atc message" << endl;
		int bitstream_size = h->getPayloadSize();
		cout << "Bitstream size is " << bitstream_size << endl;

		//copy the bitstream (MAYBE REMOVED)
		for(int i=0;i<bitstream_size;i++){
			buf[i] = bitstream[i];
		}

		DataATCMessage_t* internal_message = (DataATCMessage_t*) calloc(1, sizeof(*internal_message));
		asn_dec_rval_t rval;
		rval = uper_decode_complete(0, &asn_DEF_DataATCMessage,(void **)&internal_message, buf, MAX_DATA_ATC_MESSAGE_SIZE);
		msg = new DataATCMsg(internal_message);
		msg->setSource(h->getSrcAddr());
		msg->setDestination(h->getDstAddr());
		msg->setTcpConnection(cn);

		if(rval.code != RC_OK) {
			fprintf(stderr,
					"Broken message encoding at byte %ld\n",
					(long)rval.consumed);
			//exit(65); /* better, EX_DATAERR */
			msg = NULL;
		} else {
			//fprintf(stdout,"Printing msg as XML...\n");
			//xer_fprint(stdout, &asn_DEF_DataATCMessage, internal_message);
		}

		break;
	}

	case COOP_INFO_MESSAGE:{

		cout << "Message is COOP_INFO" << endl;

		char buf[MAX_COOP_INFO_MESSAGE_SIZE];

		int bitstream_size =  h->getPayloadSize();
		cout << "Bitstream size is " << bitstream_size << endl;

		//copy the bitstream (MAYBE REMOVED)
		for(int i=0;i<bitstream_size;i++){
			buf[i] = bitstream[i];
		}

		CooperatorInfo_t* internal_message = (CooperatorInfo_t*) calloc(1, sizeof(*internal_message));
		asn_dec_rval_t rval;
		rval = uper_decode_complete(0, &asn_DEF_CooperatorInfo,(void **)&internal_message, buf, MAX_COOP_INFO_MESSAGE_SIZE);
		msg = new CoopInfoMsg(internal_message);
		msg->setSource(h->getSrcAddr());
		msg->setDestination(h->getDstAddr());
		msg->setTcpConnection(cn);

		if(rval.code != RC_OK) {
			fprintf(stderr,
					"Broken message encoding at byte %ld\n",
					(long)rval.consumed);
			//exit(65); /* better, EX_DATAERR */
			msg = NULL;
		} else {
			//fprintf(stdout,"Printing msg as XML...\n");
			//xer_fprint(stdout, &asn_DEF_CooperatorInfo, internal_message);
		}
		break;
	}

	case STOP_MESSAGE:
	{
		cout << "Message is STOP" << endl;
		msg = new StopMsg();
		msg->setSource(h->getSrcAddr());
		msg->setDestination(h->getDstAddr());
		msg->setTcpConnection(cn);
		break;
	}

	case ACK_SLICE_MESSAGE:
	{
		cout << "Message is ACK_SLICE" << endl;

		char buf[MAX_ACK_SLICE_MESSAGE_SIZE];

		int bitstream_size =  h->getPayloadSize();
		cout << "Bitstream size is " << bitstream_size << endl;

		//copy the bitstream (MAYBE REMOVED)
		for(int i=0;i<bitstream_size;i++){
			buf[i] = bitstream[i];
		}

		ACKsliceMessage_t* internal_message = (ACKsliceMessage_t*) calloc(1, sizeof(*internal_message));
		asn_dec_rval_t rval;
		rval = uper_decode_complete(0, &asn_DEF_ACKsliceMessage,(void **)&internal_message, buf, MAX_ACK_SLICE_MESSAGE_SIZE);
		msg = new ACKsliceMsg(internal_message);
		msg->setSource(h->getSrcAddr());
		msg->setDestination(h->getDstAddr());
		msg->setTcpConnection(cn);

		if(rval.code != RC_OK) {
			fprintf(stderr,
					"Broken message encoding at byte %ld\n",
					(long)rval.consumed);
			//exit(65); /* better, EX_DATAERR */
			msg = NULL;
		} else {
			//fprintf(stdout,"Printing msg as XML...\n");
			//xer_fprint(stdout, &asn_DEF_CooperatorInfo, internal_message);
		}

		break;
	}
	//ALEXIS 11/01 ADD CAMERA MESSAGE
	case ADD_CAMERA_MESSAGE:{

		cout << "Message is ADD_CAMERA" << endl;

		char buf[MAX_ADD_CAMERA_MESSAGE_SIZE];

		int bitstream_size =  h->getPayloadSize();
		cout << "Bitstream size is " << bitstream_size << endl;

		//copy the bitstream (MAYBE REMOVED)
		for(int i=0;i<bitstream_size;i++){
			buf[i] = bitstream[i];
		}

		AddCameraMessage_t* internal_message = (AddCameraMessage_t*) calloc(1, sizeof(*internal_message));
		asn_dec_rval_t rval;
		rval = uper_decode_complete(0, &asn_DEF_AddCameraMessage,(void **)&internal_message, buf, MAX_ADD_CAMERA_MESSAGE_SIZE);
		msg = new AddCameraMsg(internal_message);
		msg->setSource(h->getSrcAddr());
		msg->setDestination(h->getDstAddr());
		msg->setTcpConnection(cn);

		if(rval.code != RC_OK) {
			fprintf(stderr,
					"Broken message encoding at byte %ld\n",
					(long)rval.consumed);
			//exit(65); /* better, EX_DATAERR */
			msg = NULL;
		} else {
			//fprintf(stdout,"Printing msg as XML...\n");
			//xer_fprint(stdout, &asn_DEF_CooperatorInfo, internal_message);
		}
		break;
	}
	//
	default:
	{
		break;
	}



	}

	delete(h);
	return msg;
}


