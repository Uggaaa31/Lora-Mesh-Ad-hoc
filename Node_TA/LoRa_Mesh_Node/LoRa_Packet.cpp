/*
 * LoRa Packet Structure Library - Implementation
 */

#include "LoRa_Packet.h"

LoRaPacketHandler::LoRaPacketHandler() {
}

// Create DATA packet with sensor payload
LoRaPacket LoRaPacketHandler::createDataPacket(uint8_t sourceID, uint8_t destID, 
                                                const SensorDataPayload& data, uint32_t seqNum) {
    LoRaPacket packet;
    packet.header.packetType = 0x01;  // PKT_TYPE_DATA
    packet.header.sourceID = sourceID;
    packet.header.destinationID = destID;
    packet.header.sequenceNum = seqNum;
    packet.header.hopCount = 0;
    packet.header.payloadLength = sizeof(SensorDataPayload);
    
    // Copy payload
    memcpy(packet.payload, &data, sizeof(SensorDataPayload));
    
    // Calculate checksum
    packet.header.checksum = calculateChecksum(packet);
    
    return packet;
}

// Create RREQ packet
LoRaPacket LoRaPacketHandler::createRREQPacket(uint8_t sourceID, const RREQPayload& rreq) {
    LoRaPacket packet;
    packet.header.packetType = 0x02;  // PKT_TYPE_RREQ
    packet.header.sourceID = sourceID;
    packet.header.destinationID = 255;  // Broadcast
    packet.header.sequenceNum = rreq.rreqID;
    packet.header.hopCount = 0;
    packet.header.payloadLength = sizeof(RREQPayload);
    
    memcpy(packet.payload, &rreq, sizeof(RREQPayload));
    packet.header.checksum = calculateChecksum(packet);
    
    return packet;
}

// Create RREP packet
LoRaPacket LoRaPacketHandler::createRREPPacket(uint8_t sourceID, uint8_t destID, 
                                               const RREPPayload& rrep, uint32_t seqNum) {
    LoRaPacket packet;
    packet.header.packetType = 0x03;  // PKT_TYPE_RREP
    packet.header.sourceID = sourceID;
    packet.header.destinationID = destID;
    packet.header.sequenceNum = seqNum;
    packet.header.hopCount = 0;
    packet.header.payloadLength = sizeof(RREPPayload);
    
    memcpy(packet.payload, &rrep, sizeof(RREPPayload));
    packet.header.checksum = calculateChecksum(packet);
    
    return packet;
}

// Create RERR packet
LoRaPacket LoRaPacketHandler::createRERRPacket(uint8_t sourceID, const RERRPayload& rerr) {
    LoRaPacket packet;
    packet.header.packetType = 0x04;  // PKT_TYPE_RERR
    packet.header.sourceID = sourceID;
    packet.header.destinationID = 255;  // Broadcast
    packet.header.sequenceNum = 0;
    packet.header.hopCount = 0;
    packet.header.payloadLength = sizeof(RERRPayload);
    
    memcpy(packet.payload, &rerr, sizeof(RERRPayload));
    packet.header.checksum = calculateChecksum(packet);
    
    return packet;
}

// Create HELLO packet
LoRaPacket LoRaPacketHandler::createHelloPacket(uint8_t sourceID, uint32_t seqNum) {
    LoRaPacket packet;
    packet.header.packetType = 0x05;  // PKT_TYPE_HELLO
    packet.header.sourceID = sourceID;
    packet.header.destinationID = 255;  // Broadcast
    packet.header.sequenceNum = seqNum;
    packet.header.hopCount = 0;
    packet.header.payloadLength = 0;
    packet.header.checksum = calculateChecksum(packet);
    return packet;
}

// Create TIMESYNC packet (Gateway -> Nodes, broadcast NTP epoch)
LoRaPacket LoRaPacketHandler::createTimeSyncPacket(uint8_t sourceID, const TimeSyncPayload& ts) {
    LoRaPacket packet;
    packet.header.packetType = 0x06;  // PKT_TYPE_TIMESYNC
    packet.header.sourceID = sourceID;
    packet.header.destinationID = 255;  // Broadcast
    packet.header.sequenceNum = 0;
    packet.header.hopCount = 0;
    packet.header.payloadLength = sizeof(TimeSyncPayload);
    memcpy(packet.payload, &ts, sizeof(TimeSyncPayload));
    packet.header.checksum = calculateChecksum(packet);
    return packet;
}

// Calculate checksum for packet
uint16_t LoRaPacketHandler::calculateChecksum(const LoRaPacket& packet) {
    uint16_t sum = 0;
    
    // Checksum dari header (skip checksum field)
    sum += packet.header.packetType;
    sum += packet.header.sourceID;
    sum += packet.header.destinationID;
    sum += packet.header.nextHop;
    sum += packet.header.hopCount;
    sum += (packet.header.sequenceNum & 0xFF);
    sum += ((packet.header.sequenceNum >> 8) & 0xFF);
    sum += ((packet.header.sequenceNum >> 16) & 0xFF);
    sum += ((packet.header.sequenceNum >> 24) & 0xFF);
    sum += packet.header.payloadLength;
    
    // Checksum dari payload
    for (int i = 0; i < packet.header.payloadLength; i++) {
        sum += packet.payload[i];
    }
    
    return sum;
}

// Validate packet checksum
bool LoRaPacketHandler::validatePacket(const LoRaPacket& packet) {
    uint16_t calculatedChecksum = calculateChecksum(packet);
    return (calculatedChecksum == packet.header.checksum);
}

// Serialize packet ke buffer
int LoRaPacketHandler::serializePacket(const LoRaPacket& packet, uint8_t* buffer, int bufferSize) {
    if (packet.header.payloadLength > MAX_PAYLOAD_SIZE) {
        return -1;  // Invalid payload length
    }

    int totalSize = sizeof(PacketHeader) + packet.header.payloadLength;
    
    if (totalSize > bufferSize) {
        return -1;  // Buffer too small
    }
    
    // Copy header
    memcpy(buffer, &packet.header, sizeof(PacketHeader));
    
    // Copy payload
    if (packet.header.payloadLength > 0) {
        memcpy(buffer + sizeof(PacketHeader), packet.payload, packet.header.payloadLength);
    }
    
    return totalSize;
}

// Deserialize buffer ke packet
bool LoRaPacketHandler::deserializePacket(const uint8_t* buffer, int length, LoRaPacket& packet) {
    if (length < sizeof(PacketHeader)) {
        return false;  // Too short
    }
    
    // Copy header
    memcpy(&packet.header, buffer, sizeof(PacketHeader));

    // Validate payload length before copying
    if (packet.header.payloadLength > MAX_PAYLOAD_SIZE) {
        return false;
    }
    
    // Validate length
    int expectedLength = sizeof(PacketHeader) + packet.header.payloadLength;
    if (length < expectedLength) {
        return false;  // Incomplete packet
    }
    
    // Copy payload
    if (packet.header.payloadLength > 0) {
        memcpy(packet.payload, buffer + sizeof(PacketHeader), packet.header.payloadLength);
    }
    
    // Validate checksum
    return validatePacket(packet);
}

// Print packet untuk debugging
void LoRaPacketHandler::printPacket(const LoRaPacket& packet) {
    Serial.println("=== Packet Info ===");
    Serial.printf("Type: %s (0x%02X)\n", getPacketTypeName(packet.header.packetType), packet.header.packetType);
    Serial.printf("Source: %d\n", packet.header.sourceID);
    Serial.printf("Destination: %d\n", packet.header.destinationID);
    Serial.printf("Next Hop: %d\n", packet.header.nextHop);
    Serial.printf("Hop Count: %d\n", packet.header.hopCount);
    Serial.printf("Seq Num: %u\n", packet.header.sequenceNum);
    Serial.printf("Payload Length: %d\n", packet.header.payloadLength);
    Serial.printf("Checksum: 0x%04X (Valid: %s)\n", 
                  packet.header.checksum, 
                  validatePacket(packet) ? "YES" : "NO");
    Serial.println("==================");
}

// Get packet type name
const char* LoRaPacketHandler::getPacketTypeName(uint8_t type) {
    switch (type) {
        case 0x01: return "DATA";
        case 0x02: return "RREQ";
        case 0x03: return "RREP";
        case 0x04: return "RERR";
        case 0x05: return "HELLO";
        case 0x06: return "TIMESYNC";
        case 0x07: return "FATIGUE_IMU";
        case 0x08: return "FATIGUE_STATUS";
        case 0x09: return "VEHICLE_TELEMETRY";
        default: return "UNKNOWN";
    }
}

// Compute simple checksum
uint16_t LoRaPacketHandler::computeChecksum(const uint8_t* data, int length) {
    uint16_t sum = 0;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}
