/*
 * S2GInterface.h
 *
 *  Created on: 15/lug/2014
 *      Author: greeneyes
 */

#ifndef S2GINTERFACE_H_
#define S2GINTERFACE_H_


#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>
#include "Messages/Header.h"

class NodeManager;
class MessageParser;

using boost::asio::ip::tcp;
using namespace std;



class S2GInterface{

public:
	S2GInterface(NodeManager* nm, MessageParser* m, boost::asio::io_service& io_service,
			tcp::resolver::iterator endpoint_iterator);
	void startInterface(){
		r_thread = boost::thread(&S2GInterface::runThread, this);
	}
	void runThread();
	void writeMsg(Message* msg);




private:
	void stop();
	void start_read_header();
	void handle_read_message(const boost::system::error_code& ec);
	void data_sent_handler(const boost::system::error_code& ec);
	void handle_connect(const boost::system::error_code& error,
			tcp::resolver::iterator endpoint_iter);
	boost::asio::io_service& io_service_;
	tcp::socket socket_;
	boost::thread r_thread;
	uchar header_[HEADER_SIZE];
	vector<int> messageId_;
	vector<int> packetId_;

	vector<uchar> out;


	NodeManager* node_manager;
	MessageParser* msg_parser;
};



#endif /* S2GINTERFACE_H_ */
