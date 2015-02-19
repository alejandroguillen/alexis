/*
 * IncomingMessageQueue.h
 *
 *  Created on: 06/giu/2014
 *      Author: antlab
 */
#include "opencv2/core/core.hpp"
#include <vector>
#include "boost/thread/condition.hpp"
#include "Messages/Message.h"
#include "Messages/StartCTAMsg.h"
#include "Messages/StartATCMsg.h"
#include "Messages/StartDATCMsg.h"
#include "Messages/DataCTAMsg.h"
#include "Messages/DataATCMsg.h"
#include "Messages/StopMsg.h"
#include "Messages/CoopInfoMsg.h"
#include "RadioSystem/RadioSystem.h"

using namespace std;

class MessageParser;

//MAIN IDEA: store incoming packets, check when all the packets needed to construct
//a message are there, then deserialize the bitstream in a Message object and pass it
//to the node manager

//the structure may also be used by multiple receiver threads (e.g. during datc),
//hence it is thread-safe

//this structure may be used for reliable transfer. Logic to append packets in the
//right order should be added.

struct message_queue_entry{
	int src_addr;
	int dst_addr;
	MessageType message_type;
	int seq_num;
	int num_packets;	//this is the number of packets that should be added
	int last_packet_id; //this is the last packet_id added
	int last_packet_id1;
	int last_packet_id2;
	//vector<int> last_packet_id_vec;
	vector<uchar> bitstream; //this is the complete ASN.1  bistream
	double start_time;
	double end_time;
};


class IncomingMessageQueue {
public:
	IncomingMessageQueue(RadioSystem* radio_system, MessageParser* m);
	virtual ~IncomingMessageQueue();
	int size();

	void addPacketToQueue(int src_addr, int dst_addr, MessageType message_type, int seq_num, int num_packets, int packet_id, vector<char> packet_bitstream);
	void addPacketToSinkQueue(int src_addr, int dst_addr, MessageType message_type, int seq_num, int num_packets, int packet_id, vector<char> packet_bitstream);
private:
	int last_seq_num;
	int last_seq_num1;
	int last_seq_num2;
	//vector<int> last_seq_num_vec;
	
	RadioSystem *radio_system_ptr;
	MessageParser *msg_parser;
	vector<message_queue_entry> message_queue;
	//mutable boost::mutex the_mutex;
	void deserializeAndNotify(int cur_pos);
};


