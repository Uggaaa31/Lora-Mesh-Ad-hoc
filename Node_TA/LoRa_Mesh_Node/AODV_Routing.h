/*
 * AODV Routing Protocol Implementation
 * For LoRa Mesh Network
 */

#ifndef AODV_ROUTING_H
#define AODV_ROUTING_H

#include <Arduino.h>
#include "LoRa_Packet.h"
#include "config.h"

#define MAX_ROUTING_ENTRIES 20

// Route Entry Status
enum RouteStatus {
    ROUTE_INVALID = 0,
    ROUTE_VALID = 1,
    ROUTE_REPAIRING = 2
};

// Routing Table Entry
struct RouteEntry {
    uint8_t destination;             // Destination node ID
    uint8_t nextHop;                 // Next hop untuk reach destination
    uint8_t hopCount;                // Number of hops
    uint32_t destinationSeq;         // Destination sequence number
    unsigned long timestamp;          // Last update time
    unsigned long lifetime;           // Route lifetime
    RouteStatus status;              // Route status
    
    RouteEntry() {
        destination = 0;
        nextHop = 0;
        hopCount = 0;
        destinationSeq = 0;
        timestamp = 0;
        lifetime = 0;
        status = ROUTE_INVALID;
    }
};

// RREQ Cache Entry (untuk prevent RREQ loops)
struct RREQCacheEntry {
    uint8_t originatorID;
    uint32_t rreqID;
    unsigned long timestamp;
    bool valid;
    
    RREQCacheEntry() {
        originatorID = 0;
        rreqID = 0;
        timestamp = 0;
        valid = false;
    }
};

// Pending RREQ Entry (untuk retry mechanism)
struct PendingRREQ {
    uint8_t destinationID;
    uint32_t rreqID;
    uint8_t retryCount;
    unsigned long timestamp;
    unsigned long rreqSentTime; // Waktu RREQ pertama dikirim (untuk hitung discovery time)
    bool active;
    
    PendingRREQ() {
        destinationID = 0;
        rreqID = 0;
        retryCount = 0;
        timestamp = 0;
        rreqSentTime = 0;
        active = false;
    }
};

class AODVRouting {
public:
    AODVRouting(uint8_t nodeID);
    
    // Initialization
    void begin();
    void update();  // Call in loop() untuk periodic tasks
    
    // Route management
    bool hasRouteTo(uint8_t destination);
    uint8_t getNextHop(uint8_t destination);
    bool addRoute(uint8_t destination, uint8_t nextHop, uint8_t hopCount, uint32_t destSeq);
    void invalidateRoute(uint8_t destination);
    void cleanupRoutes();  // Garbage collection
    
    // Sequence number management
    uint32_t getSequenceNumber();
    void incrementSequenceNumber();
    
    // RREQ handling
    bool shouldProcessRREQ(uint8_t originatorID, uint32_t rreqID);
    void addRREQToCache(uint8_t originatorID, uint32_t rreqID);
    void cleanupRREQCache();
    
    // Route discovery
    void initiateRouteDiscovery(uint8_t destinationID);
    void retryRREQ();
    
    // Packet handlers
    void handleRREQ(const LoRaPacket& packet);
    void handleRREP(const LoRaPacket& packet);
    void handleRERR(const LoRaPacket& packet);
    void handleHello(const LoRaPacket& packet);
    
    // Hello messages
    void sendHelloMessage();
    
    // Utility
    void printRoutingTable();
    void printRREQCache();
    
    // Statistik Route Discovery (Skenario 2 Skripsi)
    uint16_t routeDiscoverySuccess;  // Rute berhasil ditemukan
    uint16_t routeDiscoveryFail;     // Rute gagal (max retries habis)

    // Callbacks - set these to handle packet transmission
    void (*onSendPacket)(const LoRaPacket& packet) = nullptr;
    
private:
    uint8_t myNodeID;
    uint32_t sequenceNumber;
    uint32_t rreqIDCounter;
    
    RouteEntry routingTable[MAX_ROUTING_ENTRIES];
    RREQCacheEntry rreqCache[MAX_ROUTING_ENTRIES];
    PendingRREQ pendingRREQs[5];  // Max 5 concurrent route discoveries
    
    unsigned long lastHelloTime;
    unsigned long lastCleanupTime;
    
    // Helper methods
    int findRouteIndex(uint8_t destination);
    int findFreeRouteSlot();
    int findRREQCacheIndex(uint8_t originatorID, uint32_t rreqID);
    int findFreePendingSlot();
    
    void sendRREQ(uint8_t destinationID, uint32_t rreqID);
    void sendRREP(uint8_t originatorID, uint8_t destinationID, uint32_t destSeq, uint8_t hopCount);
    void sendRERR(uint8_t unreachableNode, uint32_t unreachableSeq);
};

#endif // AODV_ROUTING_H
