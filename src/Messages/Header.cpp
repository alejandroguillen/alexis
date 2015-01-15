/*
 * PacketHeader.cpp
 *
 *  Created on: 17/lug/2014
 *      Author: greeneyes
 */
#include "Header.h"
using namespace std;

ostream& operator <<(ostream& os, const Header header){

    os << "source address: " << header.getSrcAddr() << endl;
    os << "destination address: " << header.getDstAddr() << endl;
    os << "message type: " << header.getMsgT() << endl;
    os << "packet id: " << header.getPacketId() << endl;
    os << "number of packets: " << header.getNumPackets() << endl;
    os << "sequence number: " << header.getSeqNum() << endl;
    os << "payload size: " << header.getPayloadSize() << endl;

    return os;
}

Header::Header(vector<uchar> out){

//    short _src_addr; 	// Source address
//    short _dst_addr;	// Destination address
//    msg_type _msg_t;	// Message type (see below)
//    short _packet_id;	// Packet id
//    short _num_packets;	// Total number of packets composing the message
//    short _seq_num;     // Sequence number (for duplicates detection)

    uint16_t temp;
    uint32_t temp32;

    // Source address
    temp = 0;
    temp |= ((uint16_t) out[0]) & 0xFF;
    temp <<= 8;
    temp |= ((uint16_t) out[1]) & 0xFF;
    temp = ntohs(temp);

    _src_addr = temp;

    // Destination address
    temp = 0;
    temp |= ((uint16_t) out[2]) & 0xFF;
    temp <<= 8;
    temp |= ((uint16_t) out[3]) & 0xFF;
    temp = ntohs(temp);

    _dst_addr = temp;


    // Sequence number (for duplicates detection)
    /*temp = 0;
    temp |= ((uint8_t) out[10]) & 0xFF;
    temp <<= 8;
    temp |= ((uint16_t) out[11]) & 0xFF;
    temp = ntohs(temp);*/
    _seq_num = ((uint8_t) out[4]);

    // Total number of packets composing the message
    temp = 0;
    temp |= ((uint16_t) out[5]) & 0xFF;
    temp <<= 8;
    temp |= ((uint16_t) out[6]) & 0xFF;
    temp = ntohs(temp);

    _num_packets = temp;

    // Packet id
    temp = 0;
    temp |= ((uint16_t) out[7]) & 0xFF;
    temp <<= 8;
    temp |= ((uint16_t) out[8]) & 0xFF;
    temp = ntohs(temp);

    _packet_id = temp;


    // Message type (see below)
//    temp = 0;
//    temp |= ((uint16_t) out[4]) & 0xFF;
//    temp <<= 8;
//    temp |= ((uint16_t) out[5]) & 0xFF;
//    temp = ntohs(temp);
    _msg_t = static_cast<MessageType>(out[9]);


    // Payload size
    temp32 = 0;
    temp32 |= ((uint32_t) out[10]) & 0xFF;
    temp32 <<= 8;
    temp32 |= ((uint32_t) out[11]) & 0xFF;
    temp32 <<= 8;
    temp32 |= ((uint32_t) out[12]) & 0xFF;
    temp32 <<= 8;
    temp32 |= ((uint32_t) out[13]) & 0xFF;
    temp32 = ntohl(temp32);

    _payload_size = temp32;
}

vector<uchar> Header::serialization(){


    //    short _src_addr; 	// Source address
    //    short _dst_addr;	// Destination address
    //    msg_type _msg_t;	// Message type (see below)
    //    short _packet_id;	// Packet id
    //    short _num_packets;	// Total number of packets composing the message
    //    short _seq_num;     // Sequence number (for duplicates detection)

    vector<uchar> out;
    uint16_t temp;
    uint32_t temp32;

    // Source address
    temp = htons(_src_addr);
    out.push_back( (uchar) ((temp >> 8) & 0xFF) );
    out.push_back( (uchar) ((temp) & 0xFF) );

    // Destination address
    temp = htons(_dst_addr);
    out.push_back( (uchar) ((temp >> 8) & 0xFF) );
    out.push_back( (uchar) ((temp) & 0xFF) );

    // Sequence number (for duplicates detection)
    //temp = htons(_seq_num);
    //out.push_back( (uchar) ((temp >> 8) & 0xFF) );
    //out.push_back( (uchar) ((temp) & 0xFF) );
    out.push_back( (uchar) (_seq_num));

    // Total number of packets composing the message
    temp = htons(_num_packets);
    out.push_back( (uchar) ((temp >> 8) & 0xFF) );
    out.push_back( (uchar) ((temp) & 0xFF) );

    // Packet id
    temp = htons(_packet_id);
    out.push_back( (uchar) ((temp >> 8) & 0xFF) );
    out.push_back( (uchar) ((temp) & 0xFF) );

    // Message type
    //temp = htons((short) _msg_t);
    //out.push_back( (uchar) ((temp >> 8) & 0xFF) );
    //out.push_back( (uchar) ((temp) & 0xFF) );
    out.push_back( (uchar) _msg_t);

    // Payload size
    temp32 = htonl(_payload_size);
    out.push_back( (uchar) ((temp32 >> 24) & 0xFF) );
    out.push_back( (uchar) ((temp32 >> 16) & 0xFF) );
    out.push_back( (uchar) ((temp32 >> 8) & 0xFF) );
    out.push_back( (uchar) ((temp32) & 0xFF) );

    return out;
}
