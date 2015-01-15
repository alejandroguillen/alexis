/*
 *Header.h
 *
 *  Created on: 17/lug/2014
 *      Author: greeneyes
 */

#ifndef HEADER_H_
#define HEADER_H_

#include <iostream>
#include "Message.h"


using namespace std;

#define HEADER_SIZE 14

class Header
{
public:

    Header(){}

    Header(uint16_t src, uint16_t dst, MessageType t, uint16_t id, uint16_t num, uint8_t seqn, uint32_t payload):
        _src_addr(src),
        _dst_addr(dst),
        _msg_t(t),
        _packet_id(id),
        _num_packets(num),
        _seq_num(seqn),
        _payload_size(payload)
    {
    }

    Header(vector<uchar> a);

    vector<uchar> serialization();

    friend ostream& operator <<(ostream& os, const Header mex);

    uint16_t getDstAddr() const {
        return _dst_addr;
    }

    void setDstAddr(uint16_t dstAddr) {
        _dst_addr = dstAddr;
    }

    MessageType getMsgT() const {
        return _msg_t;
    }

    void setMsgT(MessageType msgT) {
        _msg_t = msgT;
    }

    uint16_t getNumPackets() const {
        return _num_packets;
    }

    void setNumPackets(uint16_t numPackets) {
        _num_packets = numPackets;
    }

    uint16_t getPacketId() const {
        return _packet_id;
    }

    void setPacketId(uint16_t packetId) {
        _packet_id = packetId;
    }

    uint16_t getSeqNum() const {
        return _seq_num;
    }

    void setSeqNum(uint16_t seqNum) {
        _seq_num = seqNum;
    }

    uint16_t getSrcAddr() const {
        return _src_addr;
    }

    void setSrcAddr(uint16_t srcAddr) {
        _src_addr = srcAddr;
    }

    uint32_t getPayloadSize() const{
        return _payload_size;
    }

    void setPayloadSize(const uint32_t &payload_size);

private:

    uint16_t _src_addr; 	// Source address
    uint16_t _dst_addr;	    // Destination address
    MessageType _msg_t;	    // Message type (see below)
    uint16_t _packet_id;	// Packet id
    uint16_t _num_packets;	// Total number of packets composing the message
    uint8_t _seq_num;      // Sequence number (for duplicates detection)
    uint32_t _payload_size; // Number of bytes to be read

};



#endif /* PACKETHEADER_H_ */
