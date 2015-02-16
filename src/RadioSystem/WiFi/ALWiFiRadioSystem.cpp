/*
 * ALWiFiRadioSystem.cpp
 *
 *  Created on: 12/dec/2014
 *      Author: Alejandro Guillén
 */

#include "ALWiFiRadioSystem.h"
#include <boost/bind.hpp>
#include "NodeManager/NodeManager.h"


ALWiFiRadioSystem::ALWiFiRadioSystem(tcp::resolver::query query, tcp::resolver::query query2, std::string mode, NodeManager* nm) :
io_service(),
acceptor(io_service),
connection_manager(nm),
new_connection(new Connection(io_service,
		connection_manager)),
new_connection2(new Connection(io_service,
		connection_manager)),
node_manager(nm)
{
	new_connection->setNodeManager(node_manager);
	new_connection2->setNodeManager(node_manager);
	tcp::resolver resolver(io_service);
	tcp::endpoint endpoint = *resolver.resolve(query);
	tcp::endpoint endpoint2 = *resolver.resolve(query2);
	tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
	tcp::resolver::iterator endpoint_iterator2 = resolver.resolve(query2);


	std::cout << "connecting to Camera 1..." << std::endl;
	boost::asio::async_connect(new_connection->socket(), endpoint_iterator,
			boost::bind(&ALWiFiRadioSystem::handleConnect, this,
					boost::asio::placeholders::error, endpoint_iterator, new_connection));
	std::cout << "connecting to Camera 2..." << std::endl;				
	boost::asio::async_connect(new_connection2->socket(), endpoint_iterator2,
			boost::bind(&ALWiFiRadioSystem::handleConnect, this,
					boost::asio::placeholders::error, endpoint_iterator2, new_connection2));
					

}
void ALWiFiRadioSystem::startReceiver(){
	io_service.run();
}

void ALWiFiRadioSystem::handleConnect(const boost::system::error_code& error,
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

void ALWiFiRadioSystem::handleAccept(const boost::system::error_code& e){
	if (!e)
	{
		std::cout << "connected..." << std::endl;
		connection_manager.start(new_connection);


		new_connection = new Connection(io_service,
				connection_manager);
		new_connection->setNodeManager(node_manager);
		acceptor.async_accept(new_connection->socket(),
				boost::bind(&ALWiFiRadioSystem::handleAccept, this,
						boost::asio::placeholders::error));


	}
}

std::set<Connection*> ALWiFiRadioSystem::getWiFiConnections(){
	return connection_manager.getWiFiConnections();
}
