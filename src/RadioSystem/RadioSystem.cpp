#include <stdlib.h>
#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include "RadioSystem/RadioSystem.h"
#include "NodeManager/NodeManager.h"
#include "RadioSystem/WiFi/WiFiRadioSystem.h"

using namespace std;




RadioSystem::RadioSystem(NodeManager* nm, MessageParser* m, string ip_address, string ip_address2, string port){
	nodeManager_ptr = nm;
	msg_parser = m;
	incoming_message_queue_ptr = new IncomingMessageQueue(this,msg_parser);
	NodeType node_type = nodeManager_ptr->getNodeType();

	switch(node_type){
	case SINK:
	{
		telosbRadioSystem_ptr = new TelosbRadioSystem();
		telosbRadioSystem_ptr->setIncomingMessageQueue(incoming_message_queue_ptr);
		break;
	}
	case CAMERA:
	{
		telosbRadioSystem_ptr = new TelosbRadioSystem();
		telosbRadioSystem_ptr->setIncomingMessageQueue(incoming_message_queue_ptr);
		
		tcp::resolver::query query(ip_address, port);
		wifiRadioSystem_ptr = new WiFiRadioSystem(query,std::string("server"),nodeManager_ptr);
		break;
	}
	case COOPERATOR:
	{	
		tcp::resolver::query query(ip_address, port);
		tcp::resolver::query query2(ip_address2, port);
		wifiRadioSystem_ptr = new WiFiRadioSystem(query,query2,std::string("client"),nodeManager_ptr);
		break;
	}
	default:
		break;
	}
}

void RadioSystem::startWiFiReceiver(){
	wifiRadioSystem_ptr->startReceiver();
}


int RadioSystem::startTelosbReceiver(string dev_name, string node_id){
	telosbRadioSystem_ptr->setNodeID(atoi(node_id.c_str()));
	if(telosbRadioSystem_ptr->openRadio(dev_name.c_str(),115200,0)==0){
		telosbRadioSystem_ptr->startReceiver();
		cout << "RS: telosb ready!" << endl;
		int NodeID = telosbRadioSystem_ptr->getNodeID(); //ALEXIS 11/01
		cout << "RS: camera " << NodeID << " activated" << endl; //ALEXIS 11/01
		//ALEXIS 11/01 ADD CAMERA MESSAGE
		if(NodeID != 0){
			nodeManager_ptr->AddCameraMessage(NodeID);
			cout << "" << endl;
			cout << "Camera " << NodeID << " ready to receive from sink" << endl;
		}
		//
		return 0;
	}
	cout << "RS: Error in opening radio device" << endl;
	telosbRadioSystem_ptr->startReceiver();
	return -1;
}

void RadioSystem::joinTelosbReceiver(){
	telosbRadioSystem_ptr->joinReceiver();
}


void RadioSystem::startTelosbMsg(){
	telosbRadioSystem_ptr->startMsg();
}

serial_source RadioSystem::getTelosb(){
	return telosbRadioSystem_ptr->getTelosb();
}

void RadioSystem::notifyMsg(Message* msg){
	cout << "RS: Notifying message to " << nodeManager_ptr << endl;
	nodeManager_ptr->notify_msg(msg);
}

std::set<Connection*> RadioSystem::getWiFiConnections(){
	return wifiRadioSystem_ptr->getWiFiConnections();
}

//void RadioSystem::radioSystemThread(){
//	int counter=0;
//	while(0){
//
//		for(int op=0; op<3; op++){
//			// simulate the transmission delay
//			int wait_time = 103000;
//			usleep(wait_time);
//
//			// simulate the reception of messages
//			int data1 = rand() % 10 + 1;
//			int data2 = rand() % 10 + 1;
//
//			message *msg = new message(data1,data2,op);
//			cout << "RS: Message received: " << op << "," << data1 << "," << data2 << endl;
//			// notify the message
//			nodeManager_ptr->notify_msg(msg);
//		}
//		counter++;
//		if(counter==30){
//			sleep(3);
//			counter = 0;
//		}
//	}
//
//}

