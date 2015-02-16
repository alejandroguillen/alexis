#ifndef START_DATC_MSG_H
#define START_DATC_MSG_H

#include "Message.h"

class StartDATCMsg : public Message{
private:
	StartDATCMessage_t* internal_msg;

public:
	~StartDATCMsg();
	StartDATCMsg(int fps, DetectorTypes_t det, double det_thr, DescriptorTypes_t desc, int desc_length,
			int max_feat, bool rotation_invariant, CodingChoices_t coding, bool transfer_kpt, bool transfer_scale,
            bool transfer_orientation, int num_feat_per_block, int num_cooperators);
	StartDATCMsg(StartDATCMessage_t* internal_message);

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
	void setNumFeatPerBlocks(int n); // new
	void setNumCooperators(int n);

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
	int getNumCooperators();

	int getBitStream(vector<uchar>& bitstream);
};

#endif
