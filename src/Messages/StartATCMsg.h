#ifndef START_ATC_MSG_H
#define START_ATC_MSG_H

#include "Message.h"

class StartATCMsg : public Message{
private:
	StartATCMessage_t* internal_msg;

public:
	~StartATCMsg();
	StartATCMsg(int fps, DetectorTypes_t det, double det_thr, DescriptorTypes_t desc, int desc_length,
			int max_feat, bool rotation_invariant, CodingChoices_t coding, bool transfer_kpt, bool transfer_scale,
            bool transfer_orientation, int num_feat_per_blocks);
	StartATCMsg(StartATCMessage_t* internal_message);

	void setFramesPerSecond(int fps);
	void setDetectorType(DetectorTypes_t det);
	void setDetectorThreshold(double det_thr);
	void setDescriptorType(DescriptorTypes_t desc);
	void setDescriptorLength(int desc_length);
	void setMaxNumFeat(int max_feat);
	void setRotationInvariant(bool rotation_invariant);
	void setCoding(CodingChoices_t coding);
	void setTransferKpt(bool transfer_kpt);
	void setTransferScale(bool transfer_scale);
	void setTransferOrientation(bool transfer_orientation);
	void setNumFeatPerBlock(int n); // new

	int getFramesPerSecond();
	DetectorTypes_t getDetectorType();
	double getDetectorThreshold();
	DescriptorTypes_t getDescriptorType();
	int getDescriptorLength();
	int getMaxNumFeat();
	CodingChoices_t getCoding();
	bool getTransferKpt();
	bool getTransferScale();
	bool getTransferOrientation();
	int getNumFeatPerBlock();

	int getBitStream(vector<uchar>& bitstream);
};

#endif
