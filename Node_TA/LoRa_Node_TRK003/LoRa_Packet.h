/*
 * LoRa Packet Structure Library (UNIFIED v3.0)
 * Handles packet creation, parsing, and validation
 * 
 * PENTING: File ini HARUS identik di semua folder project!
 *   - Gateway_TA/LoRa_Gateway/
 *   - Node_TA/LoRa_Mesh_Node/
 *   - Node_TA/LoRa_Node_TRK002/
 *   - Node_TA/LoRa_Node_TRK003/
 *   - Node_TA/lora_saenab/
 *   - Node_TA/lora_nailah/
 */

#ifndef LORA_PACKET_H
#define LORA_PACKET_H

#include <Arduino.h>

#define MAX_PAYLOAD_SIZE 200
#define MAX_ROUTE_PATH   5       // Maksimum node dalam route path

// ================================================================
// PACKET HEADER — Identik di semua node & gateway
// ================================================================
struct PacketHeader {
    uint8_t packetType;
    uint8_t sourceID;
    uint8_t destinationID;
    uint8_t nextHop;
    uint8_t hopCount;
    uint32_t sequenceNum;
    uint16_t payloadLength;
    uint8_t  routePathLen;              // Jumlah entry aktif di routePath
    uint8_t  routePath[MAX_ROUTE_PATH]; // ID node yang dilalui paket (src -> relay1 -> relay2 -> ...)
    uint16_t checksum;
} __attribute__((packed));

// ================================================================
// SENSOR SUB-STRUCTS
// ================================================================
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

// ================================================================
// DATA PAYLOADS — Semua payload WAJIB konsisten di semua folder
// ================================================================

// Payload utama untuk TRK-001/002/003 (sensor dummy)
struct SensorDataPayload {
    GPSData gps;
    IMUData imu;
    float batteryVoltage;
    int16_t signalStrength;
    uint32_t txTimestamp;      // Epoch ms (low 32-bit) saat node kirim — untuk hitung latensi
    uint32_t routeDiscMs;      // Waktu discovery terakhir (ms)
    uint8_t  routeHops;        // Jumlah hop terakhir
    uint8_t  padding[11];      // Tambahan padding agar tepat 70 Byte
} __attribute__((packed));

static_assert(sizeof(SensorDataPayload) <= MAX_PAYLOAD_SIZE,
              "SensorDataPayload exceeds MAX_PAYLOAD_SIZE");

// Payload IMU fatigue detection (lora_saenab → Gateway)
struct ImuFatiguePayload {
    uint8_t packetType;
    uint8_t nodeId;
    uint32_t ts;
    int16_t pitch100;
    int16_t roll100;
    int16_t gx1000;
    int16_t gy1000;
    int16_t gz1000;
    uint32_t routeDiscMs;      // Waktu discovery terakhir
    uint8_t  routeHops;        // Jumlah hop terakhir
    uint8_t  buzzerActive;     // 1 = ON, 0 = OFF (status buzzer fisik)
    uint8_t  padding[10];      // Tambahan padding agar tepat 30 Byte
} __attribute__((packed));

static_assert(sizeof(ImuFatiguePayload) <= MAX_PAYLOAD_SIZE,
              "ImuFatiguePayload exceeds MAX_PAYLOAD_SIZE");

// Payload status alarm fatigue (Gateway → lora_saenab)
struct FatigueStatusPayload {
    uint8_t packetType;
    uint8_t targetNodeId;
    uint8_t status;            // 0=NORMAL, 1=LELAH, 2=TERTIDUR
} __attribute__((packed));

static_assert(sizeof(FatigueStatusPayload) <= MAX_PAYLOAD_SIZE,
              "FatigueStatusPayload exceeds MAX_PAYLOAD_SIZE");

// Payload safety condition flags (lora_nailah → Gateway)
struct SafetyConditionPayload {
    uint8_t packetType;
    uint8_t nodeId;
    uint32_t ts;
    uint8_t flags;
    uint32_t routeDiscMs;      // Waktu discovery terakhir
    uint8_t  routeHops;        // Jumlah hop terakhir
} __attribute__((packed));

static_assert(sizeof(SafetyConditionPayload) <= MAX_PAYLOAD_SIZE,
              "SafetyConditionPayload exceeds MAX_PAYLOAD_SIZE");

// Payload vehicle telemetry lengkap (lora_nailah → Gateway, opsional)
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
    uint32_t routeDiscMs;      // Waktu discovery terakhir
    uint8_t  routeHops;        // Jumlah hop terakhir
    uint8_t  padding[23];      // Tambahan padding agar tepat 120 Byte
} __attribute__((packed));

static_assert(sizeof(VehicleTelemetryPayload) <= MAX_PAYLOAD_SIZE,
              "VehicleTelemetryPayload exceeds MAX_PAYLOAD_SIZE");

// ================================================================
// AODV ROUTING PAYLOADS
// ================================================================
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

// ================================================================
// TIME SYNC & DIAGNOSTIC PAYLOADS
// ================================================================

// Time Sync Payload — Gateway broadcast epoch ke semua Node (NTP Sync)
struct TimeSyncPayload {
    uint32_t epochSeconds;     // Unix timestamp (detik sejak 1 Jan 1970)
    uint16_t millisPart;       // Milidetik dalam detik ini (0-999)
} __attribute__((packed));

// Route Discovery Diagnostic Payload — Node -> Gateway (pengukuran waktu pembentukan rute)
struct DiscoveryDiagPayload {
    uint8_t originNodeId;      // Node yang memulai route discovery
    uint8_t targetNodeId;      // Node tujuan pencarian rute
    uint32_t rreqTimestamp;    // Epoch ms (low32) saat RREQ pertama dikirim
    uint32_t rrepTimestamp;    // Epoch ms (low32) saat RREP diterima
    uint32_t discoveryMs;      // Durasi pembentukan rute (ms)
    uint8_t hopCount;          // Jumlah hop rute yang terbentuk
    uint8_t retryCount;        // Jumlah retry RREQ sebelum berhasil
    uint8_t success;           // 1 = berhasil, 0 = gagal (timeout)
} __attribute__((packed));

static_assert(sizeof(DiscoveryDiagPayload) <= MAX_PAYLOAD_SIZE,
              "DiscoveryDiagPayload exceeds MAX_PAYLOAD_SIZE");

// START_TEST Payload — Gateway broadcast untuk mulai test SF/BW baru
struct StartTestPayload {
    uint8_t  sf;               // Spreading Factor baru (7-12)
    uint32_t bwKHz;            // Bandwidth baru dalam kHz (125 atau 250)
} __attribute__((packed));

static_assert(sizeof(StartTestPayload) <= MAX_PAYLOAD_SIZE,
              "StartTestPayload exceeds MAX_PAYLOAD_SIZE");

// ================================================================
// LORA PACKET CONTAINER
// ================================================================
struct LoRaPacket {
    PacketHeader header;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    LoRaPacket() { memset(this, 0, sizeof(LoRaPacket)); }
} __attribute__((packed));

// ================================================================
// PACKET HANDLER CLASS — Semua create methods tersedia di semua folder
// ================================================================
class LoRaPacketHandler {
public:
    LoRaPacketHandler();

    // Data packets
    static LoRaPacket createDataPacket(uint8_t sourceID, uint8_t destID,
                                       const SensorDataPayload& data, uint32_t seqNum);
    static LoRaPacket createImuFatiguePacket(uint8_t sourceID, uint8_t destID,
                                             const ImuFatiguePayload& data, uint32_t seqNum);
    static LoRaPacket createFatigueStatusPacket(uint8_t sourceID, uint8_t destID,
                                                const FatigueStatusPayload& data, uint32_t seqNum);
    static LoRaPacket createSafetyConditionPacket(uint8_t sourceID, uint8_t destID,
                                                  const SafetyConditionPayload& data, uint32_t seqNum);
    static LoRaPacket createVehicleTelemetryPacket(uint8_t sourceID, uint8_t destID,
                                                   const VehicleTelemetryPayload& data, uint32_t seqNum);
    static LoRaPacket createDiagnosticPacket(uint8_t sourceID, uint8_t destID,
                                             const DiscoveryDiagPayload& data, uint32_t seqNum);
    static LoRaPacket createStartTestPacket(uint8_t sourceID, const StartTestPayload& data);

    // AODV routing packets
    static LoRaPacket createRREQPacket(uint8_t sourceID, const RREQPayload& rreq);
    static LoRaPacket createRREPPacket(uint8_t sourceID, uint8_t destID,
                                       const RREPPayload& rrep, uint32_t seqNum);
    static LoRaPacket createRERRPacket(uint8_t sourceID, const RERRPayload& rerr);
    static LoRaPacket createHelloPacket(uint8_t sourceID, uint32_t seqNum);

    // Time sync
    static LoRaPacket createTimeSyncPacket(uint8_t sourceID, const TimeSyncPayload& ts);

    // Utility
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
