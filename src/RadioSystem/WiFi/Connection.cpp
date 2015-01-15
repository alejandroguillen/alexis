/*
 * Connection.cpp
 *
 *  Created on: 16/set/2014
 *      Author: greeneyes
 */

#include "Connection.h"
#include <vector>
#include <boost/bind.hpp>
#include "ConnectionManager.h"
#include "NodeManager/NodeManager.h"
#include "RadioSystem/MessageParser.h"


using namespace std;

Connection::Connection(boost::asio::io_service& io_service,
		ConnectionManager& manager/*, MessageHandler& handler*/)
: socket_(io_service),
  connection_manager_(manager)
/*,
    message_handler_(handler)*/
{
}

void Connection::setNodeManager(NodeManager* nm){
	node_manager = nm;
}

void Connection::setMessageParser(MessageParser* m){
	message_parser = m;
}

boost::asio::ip::tcp::socket& Connection::socket()
{
	return socket_;
}

void Connection::stop()
{
	socket_.close();
}

void Connection::readHeader()
{
	cout << "Ready to read from client..." << endl;
	//read wifi packet header and then read the rest of the message
	async_read(socket_,
			boost::asio::buffer(header_, sizeof(header_)),
			boost::bind(&Connection::handleReadMessage, this, boost::asio::placeholders::error));
}


void Connection::handleReadMessage(const boost::system::error_code& ec)
{

	cout << "Header received" << endl;


	if (!ec)
	{
		cout << "Received new message header" << endl;

		h = message_parser->parseHeader(header_);
		//Header h = parseHeader(header_);
		cout << h << endl;

		if (h->getNumPackets()>1){
			std::cout << "Error: packetization is not supported for this type of message \n";
			stop();
		}

	
		//if((node_manager->node_id > 2) || (h->getDstAddr() == node_manager->node_id)){ //ALEXIS filter, unnecessary filter
		/*if(h->getDstAddr() == node_manager->node_id){ //ALEXIS filter 11/12, unnecessary filter
			if(node_manager->node_id > 2)
				cout << "That's for me. Cooperator " << node_manager->node_id << endl; //ALEXIS
			else
				cout << "That's for me. Camera " << node_manager->node_id << endl; //ALEXIS
		*/	
			//uchar data[h->getPayloadSize()];
			readBuffer_.clear();
			readBuffer_.resize(h->getPayloadSize());
			boost::system::error_code ec;

			//read the payload
			/*socket_.read_some(
					boost::asio::buffer(data, h->getPayloadSize()), ec);*/

			async_read(socket_,boost::asio::buffer(readBuffer_, h->getPayloadSize()),
					boost::bind(&Connection::parseMessage, this,
							boost::asio::placeholders::error,
							boost::asio::placeholders::bytes_transferred));

			/*if(!ec){
				Message* msg = message_parser->parseMessage(h,data,this);
				node_manager->notify_msg(msg);
				readHeader();
			}
			else{
				connection_manager_.stop(this);
			}*/
		//}
		/*else{
			if(node_manager->node_id > 2)
				cout << "Is not for me. Is for Cooperator " << h->getDstAddr() << endl; //ALEXIS
			else
				cout << "Is not for me. Is for Camera " << h->getDstAddr() << endl; //ALEXIS
		//}*/
	}
	else{
		connection_manager_.stop(this);
	}
}

void Connection::parseMessage(const boost::system::error_code& error, size_t bytes_transferred){
	if(!error){
		Message* msg = message_parser->parseMessage(h,&readBuffer_[0],this);
		if(msg!=NULL)
			node_manager->notify_msg(msg);
		//delete(h);
		readHeader();
	}
	else{
		connection_manager_.stop(this);
	}
}


void Connection::handle_write(const boost::system::error_code& e)
{
	if (!e)
	{
		std::cout << "tx ok" << std::endl;
		// Initiate graceful connection closure.
		//boost::system::error_code ignored_ec;
		//socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignored_ec);
	}

	if (e != boost::asio::error::operation_aborted)
	{
		std::cout << "tx error" << std::endl;
		//connection_manager_.stop(this);
	}
}

void Connection::writeMsg(Message* msg){


	vector<uchar> temp1;
	vector<uchar> temp2;

	if(msg->getBitStream(temp1)<=0)
		cout << "error in getting the bitstream from msg!" << endl;

	Header h(msg->getSource(), msg->getDestination(), msg->msg_type, 0, 1, msg->getSeqNum(), temp1.size());
	temp2 = h.serialization();
	std::cout << h;

	out.clear();
	out.reserve( temp1.size() + temp2.size() ); // preallocate memory
	//write header
	out.insert( out.end(), temp2.begin(), temp2.end() );
	//write payload
	out.insert( out.end(), temp1.begin(), temp1.end() );

	size_t out_length = out.size();
	boost::asio::write(socket_, boost::asio::buffer(out, out_length));
	std::cout << "writing to: " << socket_.remote_endpoint().address().to_string() << std::endl;
	std::cout << "writing to: " << socket_.remote_endpoint().port() << std::endl;
	std::cout << out.size() << std::endl;

	/*boost::asio::async_write(socket_,
				boost::asio::buffer(out),
				boost::bind(&Connection::data_sent_handler, this, boost::asio::placeholders::error));

	std::cout << "writing to: " << socket_.remote_endpoint().address().to_string() << std::endl;
	std::cout << "writing to: " << socket_.remote_endpoint().port() << std::endl;
	std::cout << out.size() << std::endl;*/
	//if(ignored_error!=0){
	//	std::cout << "tx error" << std::endl;
	//}
}

void Connection::data_sent_handler(const boost::system::error_code& ec){

	cout << "data written..." << endl;
	if (!ec){
		//		  heartbeat_timer_.expires_from_now(boost::posix_time::seconds(1));
		// heartbeat_timer_.async_wait(boost::bind(&client::start_processing, this));
	}
	else{
		std::cout << "Error on sending data: " << ec.message() << "\n";
		stop();
	}

	cout << "ok" << endl;

}


