/*
 * ALWiFiRadioSystem.h
 *
 *  Created on: 12/dec/2014
 *      Author: Alejandro Guillén
 */

#ifndef ALWIFIRADIOSYSTEM_H_
#define ALWIFIRADIOSYSTEM_H_

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include "ConnectionManager.h"
#include "Connection.h"
#include "Messages/Header.h"

using boost::asio::ip::tcp;

class ALWiFiRadioSystem{

public:
	ALWiFiRadioSystem(boost::asio::ip::tcp::resolver::query, boost::asio::ip::tcp::resolver::query, std::string mode, NodeManager* nm);
	void startReceiver();

	std::set<Connection*> getWiFiConnections();

private:
	void handleConnect(const boost::system::error_code& error,
			tcp::resolver::iterator endpoint_iter,  Connection* connection);
	void handleAccept(const boost::system::error_code&);

	boost::asio::io_service io_service;
	tcp::acceptor acceptor;
	ConnectionManager connection_manager;
	Connection* new_connection;
	Connection* new_connection2;

	NodeManager* node_manager;
	
	//boost::thread r_thread;

};


#endif /* ALWIFIRADIOSYSTEM_H_ */
