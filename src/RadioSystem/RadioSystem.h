#ifndef RADIOSYSTEM_H
#define RADIOSYSTEM_H

#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/asio.hpp>
#include "Messages/Message.h"
#include "Telosb/serialsource.h"
#include <set>

class NodeManager;
class TelosbRadioSystem;
class WiFiRadioSystem;
class ALWiFiRadioSystem; //ALEXIS 15/12 WIFI CLASS
class IncomingMessageQueue;
class MessageParser;
using boost::asio::ip::tcp;

class RadioSystem{

public:
	RadioSystem(NodeManager *nm, MessageParser* m, string ip_address, string ip_address2, string port);
	/*void set_nodeManager(NodeManager *nm){
		nodeManager_ptr = nm;
		cout << "RS node manager set " << nm << endl;
		cout << "RS node manager... " << nodeManager_ptr << endl;
	}*/

	/*void start(){
		m_thread = boost::thread(&RadioSystem::radioSystemThread, this);
	}
	void join(){
		m_thread.join();
	}*/
	int startTelosbReceiver(string dev_name, string node_id);
	void joinTelosbReceiver();

	void startWiFiReceiver();
	void startWiFiReceiver2(); //ALEXIS 15/12 WIFI CLASS
	//int startWiFiReceiver2(); //ALEXIS 14/12 WIFI THREAD
	//void joinWifiReceiver2(); //ALEXIS 14/12 WIFI THREAD

	void startTelosbMsg();
	serial_source getTelosb();
	void notifyMsg(Message* msg);
	std::set<Connection*> getWiFiConnections();


private:

	NodeManager *nodeManager_ptr;
	TelosbRadioSystem *telosbRadioSystem_ptr;
	WiFiRadioSystem *wifiRadioSystem_ptr;
	//WiFiRadioSystem *wifiRadioSystem_ptr2; //ALEXIS 14/12 WIFI THREAD
	ALWiFiRadioSystem *wifiRadioSystem_ptr2; //ALEXIS 15/12 WIFI CLASS

	IncomingMessageQueue *incoming_message_queue_ptr;
	MessageParser *msg_parser;
	//boost::thread m_thread;
	//void radioSystemThread();

};

#endif
