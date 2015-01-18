#ifndef NODEMANAGER_H
#define NODEMANAGER_H

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>
#include <queue>
#include <vector>
#include <boost/thread/condition.hpp>
#include "RadioSystem/RadioSystem.h"
#include "TaskManager/TaskManager.h"
#include "Tasks/Tasks.h"
#include "Messages/Message.h"
#include "S2GInterface/S2GInterface.h"

class OffloadingManager;
class ProcessingManager;

//todo: param structures
typedef struct CTA_param{
	int quality_factor;
	int num_slices;
	//.... other params such as num slices (w and h?)
} CTA_param_t;

typedef struct ATC_param{
	int max_features;
	DetectorTypes_t det;
	DescriptorTypes_t desc;
	double detection_threshold;
	int desc_length;
	bool rotation_invariant;
	CodingChoices_t coding;
	bool transmit_keypoints;
	bool transmit_scale;
	bool transmit_orientation;
	int num_feat_per_block;
	// .... other params such as num blocks
} ATC_param_t;

typedef struct DATC_param{
	int max_features;
	DetectorTypes_t det;
	DescriptorTypes_t desc;
	double detection_threshold;
	int desc_length;
	bool rotation_invariant;
	CodingChoices_t coding;
	bool transmit_keypoints;
	bool transmit_scale;
	bool transmit_orientation;
	int num_feat_per_block;
	int num_cooperators;
} DATC_param_t;

enum NodeType{
	SINK,
	CAMERA,
	COOPERATOR
};

//todo: states
enum SystemState{
	ACTIVE,
	IDLE,
	WAITING_LOAD
};


class NodeManager{

public:

	NodeManager(NodeType node_type, string ID);
	void set_radioSystem(RadioSystem *rs);
	void set_taskManager(TaskManager *tm);
	void set_s2gInterface(S2GInterface *s2g);

	void notify_msg(Message *msg);
	int notify_endTask();
	void deleteMsg(Message *msg);
	int received_notifications;

	void sendMessage(Message* msg);

	NodeType getNodeType();
	void notifyCooperatorOnline(Connection* conn);
	void notifyCooperatorOffline(Connection* conn);
	void notifyOffloadingCompleted(vector<KeyPoint>& kpts,Mat& features, double detTime, double descTime);
	//void sendTestPacket(Message* msg);
	int node_id; //ALEXIS
	void AddCameraMessage(int cameraid); //ALEXIS 11/01 ADD CAMERA MESSAGE
private:
	SystemState cur_state;
	NodeType node_type;

	CTA_param_t cta_param;
	ATC_param_t atc_param;
	DATC_param_t datc_param;
	std::vector<DATC_param_t> datc_param_camera;

	unsigned short outgoing_msg_seq_num;
	int frame_id;

	RadioSystem *radioSystem_ptr;
	TaskManager *taskManager_ptr;
	S2GInterface *s2gInterface_ptr;

	boost::condition cur_task_finished;

	void CTA_processing_thread();
	void ATC_processing_thread();
	void DATC_processing_thread();
	void DATC_processing_thread_cooperator(DataCTAMsg* msg);
	void DATC_store_features(DataATCMsg* msg);

	ImageAcquisition *imgAcq;
	VisualFeatureExtraction *extractor;
	VisualFeatureEncoding *encoder;
	VisualFeatureDecoding *decoder;
	OffloadingManager *offloading_manager;
	ProcessingManager *processing_manager;
	
	boost::thread p_thread;
};

#endif
