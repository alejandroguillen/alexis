/*
 * S2GInterface.cpp
 *
 *  Created on: 15/lug/2014
 *      Author: greeneyes
 */

#include "S2GInterface.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "NodeManager/NodeManager.h"
#include "RadioSystem/MessageParser.h"

S2GInterface::S2GInterface(NodeManager* nm, MessageParser* m, boost::asio::io_service& io_service,
		tcp::resolver::iterator endpoint_iterator)
: io_service_(io_service),
  socket_(io_service)
{
	node_manager = nm;
	msg_parser = m;
	cout << "connecting..." << endl;
	boost::asio::async_connect(socket_, endpoint_iterator,
			boost::bind(&S2GInterface::handle_connect, this,
					boost::asio::placeholders::error, endpoint_iterator));
}

void S2GInterface::runThread(){
	io_service_.run();
}

void S2GInterface::stop()
{
	socket_.close();
}

void S2GInterface::handle_connect(const boost::system::error_code& error,
		tcp::resolver::iterator endpoint_iter)
{
	if (!socket_.is_open())
	{
		std::cout << "Connect timed out\n";
	}

	else if (error)
	{
		std::cout << "Connect error: " << error.message() << "\n";

		socket_.close();
	}
	// Otherwise we have successfully established a connection.
	else
	{
		std::cout << "Connected to " << endpoint_iter->endpoint() << "\n";

		// Start reading the header
		cout << "Start reading..." << endl;
		start_read_header();
	}
}

void S2GInterface::start_read_header()
{
	// Set a deadline for the read operation.
	// deadline_.expires_from_now(boost::posix_time::seconds(30));

	// Start an asynchronous operation to read a newline-delimited message.

	cout << "waiting for new messages from GUI..." << endl << endl;

	socket_.async_read_some(
			boost::asio::buffer(header_, sizeof(header_)),
			boost::bind(&S2GInterface::handle_read_message, this, boost::asio::placeholders::error));

}

void S2GInterface::handle_read_message(const boost::system::error_code& ec)
{

	cout << "Header received" << endl;


	if (!ec)
	{
		cout << "Received new message header" << endl;

		//Header h = parseHeader(header_);
		//cout << h << endl;
		Header* h = msg_parser->parseHeader(header_);

		if (h->getNumPackets()>1){
			std::cout << "Error: packetization is not supported for this type of message \n";
			stop();
		}

		uchar data[h->getPayloadSize()];
		boost::system::error_code ec;

		//read the payload
		socket_.read_some(
				boost::asio::buffer(data, h->getPayloadSize()), ec);

		if(!ec){
			Message* msg = msg_parser->parseMessage(h,data);
			if(msg!=NULL)
				node_manager->notify_msg(msg);

			start_read_header();

		}
		else{
			std::cout << "Error on receive: " << ec.message() << "\n";
			stop();
		}
	}
	else{
		std::cout << "Error on receive: " << ec.message() << "\n";
		stop();
	}
}

void S2GInterface::writeMsg(Message* msg){

	vector<uchar> temp1;
	vector<uchar> temp2;

	msg->getBitStream(temp1);

	Header h(msg->getSource(), msg->getDestination(), msg->msg_type, 0, 1, msg->getSeqNum(), temp1.size());

	temp2 = h.serialization();

	out.clear();
	out.reserve( temp1.size() + temp2.size() ); // preallocate memory
	//write header
	out.insert( out.end(), temp2.begin(), temp2.end() );
	//write payload
	out.insert( out.end(), temp1.begin(), temp1.end() );


	boost::asio::async_write(socket_,
			boost::asio::buffer(out),
			boost::bind(&S2GInterface::data_sent_handler, this, boost::asio::placeholders::error));

	delete(msg);
}

void S2GInterface::data_sent_handler(const boost::system::error_code& ec){

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



