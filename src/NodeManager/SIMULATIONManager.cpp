/*
 * SIMULATIONManager.cpp
 *
 *  Created on: 02/jan/2015
 *      Author: Alejandro Guillen
 */
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "NodeManager/NodeManager.h"
#include "Tasks/Tasks.h"
#include "Messages/DataATCMsg.h"
#include "Messages/StartDATCMsg.h"
#include "Messages/CoopInfoMsg.h"
#include "Messages/ACKsliceMsg.h"
#include "Messages/AddCameraMsg.h"
#include "RadioSystem/OffloadingManager.h"
#include "NodeManager/SIMULATIONManager.h"


using namespace std;
using namespace cv;

SIMULATIONManager::SIMULATIONManager(RadioSystem* rs, NodeManager* nm){
	{

		simulationcond = false;
		//processempty=true;
		node_manager = nm;
		radio_system = rs;
		p_thread = boost::thread(&SIMULATIONManager::SIMULATION_thread, this);

	}
}

void SIMULATIONManager::start(){

		//boost::mutex::scoped_lock locksimulation(simulation_mutex);
		simulationcond = true;
		simulation_condition.notify_one();

}

void SIMULATIONManager::SIMULATION_thread(){
while(1){
	boost::mutex::scoped_lock locksimulation(simulation_mutex);
	while(simulationcond == false){
		simulation_condition.wait(locksimulation);
	}
	cout << "SM: Sending STARTDATC msg " << endl;
	simulationcond=false;
	
	sleep(2);
	//StartDATCMsg (FramesPerSecond, DetectorType, DetectorThreshold, DescriptorType, DescriptorLength,
	//				 MaxFeatures, RotationInvariant, CodingChoice, TransmitKpts, TransmitScale, 
	//				  TransmitOrientation, NumFeaPerBlock, NumCooperators)
	StartDATCMsg *datc_msg = new StartDATCMsg(0, 5, 40, 3, 512, 50, 1, 0, 1,1,1, 20,1);
	//msg(msg_type,seq_num, src_addr, dest_addr, connection)
	datc_msg->setSource(0);
	datc_msg->setDestination(node_manager->node_id);
	radio_system->notifyMsg(datc_msg);
	
}
}
