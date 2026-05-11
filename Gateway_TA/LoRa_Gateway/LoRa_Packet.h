/*
 * LoRa Packet Structure Library
 * Handles packet creation, parsing, and validation
 */

#ifndef LORA_PACKET_H
#define LORA_PACKET_H

#include <Arduino.h>

#define MAX_PAYLOAD_SIZE 200

struct PacketHeader {
    uint8_t packetType;
    uint8_t sourceID;
    uint8_t destinationID;
    uint8_t nextHop;
    uint8_t hopCount;
    uint32_t sequenceNum;
    uint16_t payloadLength;
    uint16_t checksum;
} __attribute__((packed));

struct GPSData {
    float latitude;
    float longitude;
    float altitude;
    uint32_t timestamp;
} __attribute__((packed));

struct IMUData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    uint32_t timestamp;
} __attribute__((packed));

struct SensorDataPayload {
    char nodeID[16];
    GPSData gps;
    IMUData imu;
    float batteryVoltage;
    int16_t signalStrength;
    uint32_t txTimestamp;   // Epoch ms (low 32-bit) saat node kirim — untuk hitung latensi
} __attribute__((packed));

static_assert(sizeof(SensorDataPayload) <= MAX_PAYLOAD_SIZE,
              "SensorDataPayload exceeds MAX_PAYLOAD_SIZE");

struct ImuFatiguePayload {
    uint8_t packetType;
    uint8_t nodeId;
    uint32_t ts;
    int16_t pitch100;
    int16_t roll100;
    int16_t gx1000;
    int16_t gy1000;
    int16_t gz1000;
} __attribute__((packed));

static_assert(sizeof(ImuFatiguePayload) <= MAX_PAYLOAD_SIZE,
              "ImuFatiguePayload exceeds MAX_PAYLOAD_SIZE");

struct FatigueStatusPayload {
    uint8_t packetType;
    uint8_t targetNodeId;
    uint8_t status;
} __attribute__((packed));

static_assert(sizeof(FatigueStatusPayload) <= MAX_PAYLOAD_SIZE,
              "FatigueStatusPayload exceeds MAX_PAYLOAD_SIZE");

struct SafetyConditionPayload {
    uint8_t packetType;
    uint8_t nodeId;
    uint32_t ts;
    uint8_t flags;
} __attribute__((packed));

static_assert(sizeof(SafetyConditionPayload) <= MAX_PAYLOAD_SIZE,
              "SafetyConditionPayload exceeds MAX_PAYLOAD_SIZE");

struct VehicleTelemetryPayload {
    double latitude;
    double longitude;
    float headingDeg;
    float speedMps;
    uint8_t drActive;
    uint8_t gpsValid;
    float gpsSpeed;
    float gpsHeading;
    float hdop;
    uint16_t satellites;
    float accelForward;
    float ax;
    float ay;
    float az;
    float mx;
    float my;
    float mz;
    float pitch;
    float yaw;
    float roll;
    float dt;
    uint8_t calSys;
    uint8_t calGyro;
    uint8_t calAccel;
    uint8_t calMag;
    uint8_t rollover;
    uint8_t rolloverRisk;
    uint8_t harshBraking;
    uint8_t overspeed;
    uint32_t txTimestamp;
} __attribute__((packed));

static_assert(sizeof(VehicleTelemetryPayload) <= MAX_PAYLOAD_SIZE,
              "VehicleTelemetryPayload exceeds MAX_PAYLOAD_SIZE");

struct RREQPayload {
    uint8_t originatorID;
    uint8_t destinationID;
    uint32_t rreqID;
    uint32_t originatorSeq;
    uint32_t destinationSeq;
    uint8_t hopCount;
} __attribute__((packed));

struct RREPPayload {
    uint8_t originatorID;
    uint8_t destinationID;
    uint32_t destinationSeq;
    uint8_t hopCount;
    uint32_t lifetime;
} __attribute__((packed));

struct RERRPayload {
    uint8_t unreachableNodeID;
    uint32_t unreachableSeq;
} __attribute__((packed));

// Time Sync Payload — Gateway broadcast epoch ke semua Node (P1: NTP Sync)
struct TimeSyncPayload {
    uint32_t epochSeconds;  // Unix timestamp (detik sejak 1 Jan 1970)
    uint16_t millisPart;    // Milidetik dalam detik ini (0-999)
} __attribute__((packed));

struct LoRaPacket {
    PacketHeader header;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    LoRaPacket() { memset(this, 0, sizeof(LoRaPacket)); }
} __attribute__((packed));

class LoRaPacketHandler {
public:
    LoRaPacketHandler();
    static LoRaPacket createDataPacket(uint8_t sourceID, uint8_t destID,
                                       const SensorDataPayload& data, uint32_t seqNum);
    static LoRaPacket createRREQPacket(uint8_t sourceID, const RREQPayload& rreq);
    static LoRaPacket createRREPPacket(uint8_t sourceID, uint8_t destID,
                                       const RREPPayload& rrep, uint32_t seqNum);
    static LoRaPacket createRERRPacket(uint8_t sourceID, const RERRPayload& rerr);
    static LoRaPacket createHelloPacket(uint8_t sourceID, uint32_t seqNum);
    static LoRaPacket createTimeSyncPacket(uint8_t sourceID, const TimeSyncPayload& ts);
    static LoRaPacket createImuFatiguePacket(uint8_t sourceID, uint8_t destID,
                                             const ImuFatiguePayload& data, uint32_t seqNum);
    static LoRaPacket createFatigueStatusPacket(uint8_t sourceID, uint8_t destID,
                                                const FatigueStatusPayload& data, uint32_t seqNum);
    static LoRaPacket createVehicleTelemetryPacket(uint8_t sourceID, uint8_t destID,
                                                   const VehicleTelemetryPayload& data,
                                                   uint32_t seqNum);

    static bool validatePacket(const LoRaPacket& packet);
    static uint16_t calculateChecksum(const LoRaPacket& packet);
    static int serializePacket(const LoRaPacket& packet, uint8_t* buffer, int bufferSize);
    static bool deserializePacket(const uint8_t* buffer, int length, LoRaPacket& packet);
    static void printPacket(const LoRaPacket& packet);
    static const char* getPacketTypeName(uint8_t type);

private:
    static uint16_t computeChecksum(const uint8_t* data, int length);
};

#endif // LORA_PACKET_H
