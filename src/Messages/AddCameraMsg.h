/*
 * AddCameraMsg.h
 *
 *  Created on: 11/01/2015
 *      Author: Alejandro Guillen
 */

#ifndef ADDCAMERAMSG_H_
#define ADDCAMERAMSG_H_
#include "Message.h"
#include <string>

class AddCameraMsg : public Message{
private:
	AddCameraMessage_t* internal_msg;

public:
	~AddCameraMsg();
	AddCameraMsg(int cameraid);
	AddCameraMsg(AddCameraMessage_t* internal_message);
	int getBitStream(vector<uchar>& bitstream);

	void setCameraID(int cameraid);
	int getCameraID();

};


#endif /* ADDCAMERAMSG_H_ */