/*
 * WiFiRadioSystem.h
 *
 *  Created on: 16/set/2014
 *      Author: greeneyes
 */

#ifndef WIFIRADIOSYSTEM_H_
#define WIFIRADIOSYSTEM_H_

//#include <stdio.h> //ALEXIS 14/12 WIFI THREAD
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
//#include <boost/thread.hpp> //ALEXIS 14/12 WIFI THREAD
//#include <boost/date_time.hpp> //ALEXIS 14/12 WIFI THREAD
//#include <boost/thread/condition.hpp> //ALEXIS 14/12 WIFI THREAD
#include "ConnectionManager.h"
#include "Connection.h"
#include "Messages/Header.h"

using boost::asio::ip::tcp;

class WiFiRadioSystem{

public:
	WiFiRadioSystem(boost::asio::ip::tcp::resolver::query, std::string mode, NodeManager* nm);
	void startReceiver();
	/*//ALEXIS 14/12 WIFI THREAD
	void startReceiver2(){
			r_thread = boost::thread(&WiFiRadioSystem::handleConnect, this);
		}
	void joinReceiver2(){
			r_thread.join();
		}
	//*/
	std::set<Connection*> getWiFiConnections();

private:
	void handleConnect(const boost::system::error_code& error,
			tcp::resolver::iterator endpoint_iter,  Connection* connection);
	void handleAccept(const boost::system::error_code&);

	boost::asio::io_service io_service;
	tcp::acceptor acceptor;
	ConnectionManager connection_manager;
	Connection* new_connection;

	NodeManager* node_manager;

	
};


#endif /* WIFIRADIOSYSTEM_H_ */
