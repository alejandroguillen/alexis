#include <stdlib.h>
#include <iostream>
#include <boost/asio.hpp>
#include <vector> //ALEXIS 15/12 WIFI
#include "RadioSystem/RadioSystem.h"
#include "NodeManager/NodeManager.h"
#include "NodeManager/SIMULATIONManager.h"
#include "RadioSystem/WiFi/WiFiRadioSystem.h"
#include "RadioSystem/WiFi/ALWiFiRadioSystem.h" //ALEXIS 15/12 WIFI CLASS

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
		//simulation_manager = new SIMULATIONManager(this);
		break;
	}
	case COOPERATOR:
	{	
		tcp::resolver::query query(ip_address, port);
		//wifiRadioSystem_ptr = new WiFiRadioSystem(query,std::string("client"),nodeManager_ptr); ORIGINAL
		//if(ip_address != ip_address2)	//ALEXIS 15/12 WIFI
			//tcp::resolver::query query2(ip_address2, port); //ALEXIS 14/12 WIFI
		//if(ip_address != ip_address2){ //ALEXIS 15/12 WIFI
			//tcp::resolver::query query2(ip_address2, port); //ALEXIS 15/12 WIFI
			//wifiRadioSystem_ptr2 = new WiFiRadioSystem(query2,std::string("client"),nodeManager_ptr); //ALEXIS 14/12 WIFI THREAD
		//}
		
		//ALEXIS 15/12 WIFI CONNECTION
		tcp::resolver::query query2(ip_address2, port);
		wifiRadioSystem_ptr2 = new ALWiFiRadioSystem(query,query2,std::string("client"),nodeManager_ptr);
		//
		
		break;
	}
	default:
		break;
	}
}

void RadioSystem::startWiFiReceiver(){
	wifiRadioSystem_ptr->startReceiver();
}
void RadioSystem::startWiFiReceiver2(){
	wifiRadioSystem_ptr2->startReceiver();
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

//ALEXIS SIMULATION 03/02S

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
	return wifiRadioSystem_ptr2->getWiFiConnections(); //ALEXIS 16/12 WIFI CLASS
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

