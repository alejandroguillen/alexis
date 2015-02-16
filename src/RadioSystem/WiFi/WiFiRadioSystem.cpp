/*
 * WiFiRadioSystem.cpp
 *
 *  Created on: 16/set/2014
 *      Author: greeneyes
 */

#include "WiFiRadioSystem.h"
#include <boost/bind.hpp>
#include "NodeManager/NodeManager.h"


WiFiRadioSystem::WiFiRadioSystem(tcp::resolver::query query, std::string mode, NodeManager* nm) :
io_service(),
acceptor(io_service),
connection_manager(nm),
new_connection(new Connection(io_service,
		connection_manager)),
node_manager(nm)
{
	new_connection->setNodeManager(node_manager);
	tcp::resolver resolver(io_service);
	tcp::endpoint endpoint = *resolver.resolve(query);
	tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

	if(mode.compare("server") == 0){
		//int node_id = nm->node_id;
		//open the acceptor
		std::cout << "server initializing..." << std::endl;
		acceptor.open(endpoint.protocol());
		acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
		acceptor.bind(endpoint);
		acceptor.listen();
		acceptor.async_accept(new_connection->socket(),
				boost::bind(&WiFiRadioSystem::handleAccept, this,
						boost::asio::placeholders::error));
	}

	if(mode.compare("client") == 0){
		std::cout << "connecting..." << std::endl;
		boost::asio::async_connect(new_connection->socket(), endpoint_iterator,
				boost::bind(&WiFiRadioSystem::handleConnect, this,
						boost::asio::placeholders::error, endpoint_iterator, new_connection));
	}
}

void WiFiRadioSystem::startReceiver(){
	io_service.run();
}

void WiFiRadioSystem::handleConnect(const boost::system::error_code& error,
		tcp::resolver::iterator endpoint_iter, Connection* connection){
	if (!connection->socket().is_open())
	{
		std::cout << "Connect timed out\n";
	}

	else if (error)
	{
		std::cout << "Connect error: " << error.message() << "\n";

		connection->socket().close();
	}
	// Otherwise we have successfully established a connection.
	else
	{
		std::cout << "Connected to " << endpoint_iter->endpoint() << "\n";

		// Start reading the header
		std::cout << "Start reading..." << std::endl;
		connection_manager.start(connection);
		//start_read_header();
	}
}

void WiFiRadioSystem::handleAccept(const boost::system::error_code& e){
	if (!e)
	{
		std::cout << "connected..." << std::endl;
		connection_manager.start(new_connection);


		new_connection = new Connection(io_service,
				connection_manager);
		new_connection->setNodeManager(node_manager);
		acceptor.async_accept(new_connection->socket(),
				boost::bind(&WiFiRadioSystem::handleAccept, this,
						boost::asio::placeholders::error));


	}
}

std::set<Connection*> WiFiRadioSystem::getWiFiConnections(){
	return connection_manager.getWiFiConnections();
}

