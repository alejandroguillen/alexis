#ifndef START_CTA_MSG_H
#define START_CTA_MSG_H


#include "Message.h"

class StartCTAMsg : public Message{
private:
	StartCTAMessage_t* internal_msg;

public:
	~StartCTAMsg();
    StartCTAMsg(int fps, int qf, int h, int w, int num_slices);
	StartCTAMsg(StartCTAMessage_t* internal_message);

	void setFramesPerSecond(int fps);
	void setQualityFactor(int qf);
	void setFrameHeight(int h);
	void setFrameWidth(int w);
	void setNumSlices(int n); // new

	int getFramesPerSecond();
	int getQualityFactor();
	int getFrameHeight();
	int getFrameWidth();
	int getNumSlices(); // new

	int getBitStream(vector<uchar>& bitstream);
};

#endif
