/*
 * AODV Routing Protocol Implementation
 */

#include "AODV_Routing.h"

AODVRouting::AODVRouting(uint8_t nodeID) {
    myNodeID = nodeID;
    sequenceNumber = 0;
    rreqIDCounter = 0;
    lastHelloTime = 0;
    lastCleanupTime = 0;
}

void AODVRouting::begin() {
    // Initialize routing table
    for (int i = 0; i < MAX_ROUTING_ENTRIES; i++) {
        routingTable[i].status = ROUTE_INVALID;
    }
    
    // Initialize RREQ cache
    for (int i = 0; i < MAX_ROUTING_ENTRIES; i++) {
        rreqCache[i].originatorID = 0;
        rreqCache[i].rreqID = 0;
        rreqCache[i].timestamp = 0;
        rreqCache[i].valid = false;
    }
    
    // Initialize pending RREQs
    for (int i = 0; i < 5; i++) {
        pendingRREQs[i].active = false;
    }
    
    // Inisialisasi statistik route discovery (Skenario 2)
    routeDiscoverySuccess = 0;
    routeDiscoveryFail = 0;
    for (int i = 0; i < MAX_NODES; i++) {
        lastSuccessfulDiscoveryMs[i] = 0;
        lastSuccessfulDiscoveryHops[i] = 0;
    }
    
    Serial.println("AODV Routing initialized for Node " + String(myNodeID));
}

void AODVRouting::update() {
    unsigned long now = millis();
    
    // Send periodic hello messages
    // Send periodic hello messages
    // Add Random Jitter: Â±20% of interval
    static unsigned long nextHelloTime = millis() + random(2000, 10000); // Initial random delay
    if (now > nextHelloTime) {
        sendHelloMessage();
        // Set next hello time: Interval + Random(-2000ms to +5000ms)
        nextHelloTime = now + HELLO_INTERVAL + random(-5000, 5000);
    }
    
    // Cleanup old routes and RREQ cache
    if (now - lastCleanupTime > ROUTE_CLEANUP_INTERVAL) {
        cleanupRoutes();
        cleanupRREQCache();
        lastCleanupTime = now;
    }
    
    // Retry pending RREQs
    retryRREQ();
}

// Check if route exists to destination
bool AODVRouting::hasRouteTo(uint8_t destination) {
    int idx = findRouteIndex(destination);
    if (idx >= 0 && routingTable[idx].status == ROUTE_VALID) {
        // Check if route has expired
        unsigned long now = millis();
        if (now - routingTable[idx].timestamp < routingTable[idx].lifetime) {
            return true;
        } else {
            routingTable[idx].status = ROUTE_INVALID;
        }
    }
    return false;
}

// Get next hop for destination
uint8_t AODVRouting::getNextHop(uint8_t destination) {
    int idx = findRouteIndex(destination);
    if (idx >= 0 && routingTable[idx].status == ROUTE_VALID) {
        unsigned long now = millis();
        if (now - routingTable[idx].timestamp < routingTable[idx].lifetime) {
            // Extend active route lifetime on use.
            routingTable[idx].timestamp = now;
            return routingTable[idx].nextHop;
        }
        routingTable[idx].status = ROUTE_INVALID;
    }
    return BROADCAST_ADDR;  // No route
}

uint8_t AODVRouting::getRouteHopCount(uint8_t destination) {
    int idx = findRouteIndex(destination);
    if (idx >= 0 && routingTable[idx].status == ROUTE_VALID) {
        unsigned long now = millis();
        if (now - routingTable[idx].timestamp < routingTable[idx].lifetime) {
            return routingTable[idx].hopCount;
        }
        routingTable[idx].status = ROUTE_INVALID;
    }
    return BROADCAST_ADDR;
}

// Add or update route
bool AODVRouting::addRoute(uint8_t destination, uint8_t nextHop, uint8_t hopCount, uint32_t destSeq) {
    int idx = findRouteIndex(destination);
    
    // Check if we should update existing route
    if (idx >= 0) {
        // Update only if new route has higher seq number or same seq with lower hop count
        if (destSeq > routingTable[idx].destinationSeq ||
            (destSeq == routingTable[idx].destinationSeq && hopCount < routingTable[idx].hopCount)) {
            
            routingTable[idx].nextHop = nextHop;
            routingTable[idx].hopCount = hopCount;
            routingTable[idx].destinationSeq = destSeq;
            routingTable[idx].timestamp = millis();
            routingTable[idx].lifetime = ROUTE_TIMEOUT;
            routingTable[idx].status = ROUTE_VALID;
            
            Serial.printf("Route updated: dest=%d, nextHop=%d, hops=%d\n", 
                         destination, nextHop, hopCount);
            return true;
        }
        // Keep an active route alive if the same path is seen again.
        if (destSeq == routingTable[idx].destinationSeq &&
            hopCount == routingTable[idx].hopCount &&
            nextHop == routingTable[idx].nextHop) {
            routingTable[idx].timestamp = millis();
            routingTable[idx].lifetime = ROUTE_TIMEOUT;
            routingTable[idx].status = ROUTE_VALID;
            return true;
        }
        return false;  // Existing route is better
    }
    
    // Add new route
    idx = findFreeRouteSlot();
    if (idx >= 0) {
        routingTable[idx].destination = destination;
        routingTable[idx].nextHop = nextHop;
        routingTable[idx].hopCount = hopCount;
        routingTable[idx].destinationSeq = destSeq;
        routingTable[idx].timestamp = millis();
        routingTable[idx].lifetime = ROUTE_TIMEOUT;
        routingTable[idx].status = ROUTE_VALID;
        
        Serial.printf("Route added: dest=%d, nextHop=%d, hops=%d\n", 
                     destination, nextHop, hopCount);
        return true;
    }
    
    Serial.println("ERROR: Routing table full!");
    return false;
}

// Invalidate route
void AODVRouting::invalidateRoute(uint8_t destination) {
    int idx = findRouteIndex(destination);
    if (idx >= 0) {
        routingTable[idx].status = ROUTE_INVALID;
        Serial.println("Route invalidated: dest=" + String(destination));
    }
}

// Cleanup expired routes
void AODVRouting::cleanupRoutes() {
    unsigned long now = millis();
    int cleaned = 0;
    
    for (int i = 0; i < MAX_ROUTING_ENTRIES; i++) {
        if (routingTable[i].status == ROUTE_VALID) {
            if (now - routingTable[i].timestamp > routingTable[i].lifetime) {
                routingTable[i].status = ROUTE_INVALID;
                cleaned++;
            }
        }
    }
    
    if (cleaned > 0) {
        Serial.println("Cleaned " + String(cleaned) + " expired routes");
    }
}

// Get current sequence number
uint32_t AODVRouting::getSequenceNumber() {
    return sequenceNumber;
}

// Increment sequence number
void AODVRouting::incrementSequenceNumber() {
    sequenceNumber++;
}

// Check if RREQ should be processed (atau sudah pernah diprocess)
bool AODVRouting::shouldProcessRREQ(uint8_t originatorID, uint32_t rreqID) {
    int idx = findRREQCacheIndex(originatorID, rreqID);
    return (idx < 0);  // Process jika belum ada di cache
}

// Add RREQ to cache
void AODVRouting::addRREQToCache(uint8_t originatorID, uint32_t rreqID) {
    // Find free slot first
    for (int i = 0; i < MAX_ROUTING_ENTRIES; i++) {
        if (!rreqCache[i].valid) {
            rreqCache[i].originatorID = originatorID;
            rreqCache[i].rreqID = rreqID;
            rreqCache[i].timestamp = millis();
            rreqCache[i].valid = true;
            return;
        }
    }

    // No free slot: replace oldest entry
    int oldestIdx = 0;
    unsigned long oldestTime = rreqCache[0].timestamp;
    
    for (int i = 0; i < MAX_ROUTING_ENTRIES; i++) {
        if (rreqCache[i].timestamp < oldestTime) {
            oldestIdx = i;
            oldestTime = rreqCache[i].timestamp;
        }
    }
    
    // Replace oldest entry
    rreqCache[oldestIdx].originatorID = originatorID;
    rreqCache[oldestIdx].rreqID = rreqID;
    rreqCache[oldestIdx].timestamp = millis();
    rreqCache[oldestIdx].valid = true;
}

// Cleanup old RREQ cache entries
void AODVRouting::cleanupRREQCache() {
    unsigned long now = millis();
    int cleaned = 0;
    
    for (int i = 0; i < MAX_ROUTING_ENTRIES; i++) {
        if (rreqCache[i].valid) {
            if (now - rreqCache[i].timestamp > 30000) {  // 30 seconds
                rreqCache[i].valid = false;
                rreqCache[i].originatorID = 0;
                rreqCache[i].rreqID = 0;
                rreqCache[i].timestamp = 0;
                cleaned++;
            }
        }
    }
    
    if (cleaned > 0) {
        Serial.println("Cleaned " + String(cleaned) + " RREQ cache entries");
    }
}

// Initiate route discovery
void AODVRouting::initiateRouteDiscovery(uint8_t destinationID) {
    // Optimization: Check if we are already discovering this route
    for (int i = 0; i < 5; i++) {
        if (pendingRREQs[i].active && pendingRREQs[i].destinationID == destinationID) {
            Serial.printf("DEBUG: Route discovery already in progress for dest=%d. Skipping.\n", destinationID);
            return;
        }
    }

    // Find free pending slot
    int slot = findFreePendingSlot();
    if (slot < 0) {
        Serial.println("ERROR: Cannot start route discovery, all slots busy");
        return;
    }
    
    rreqIDCounter++;
    
    pendingRREQs[slot].destinationID = destinationID;
    pendingRREQs[slot].rreqID = rreqIDCounter;
    pendingRREQs[slot].retryCount = 0;
    pendingRREQs[slot].timestamp = millis();
    pendingRREQs[slot].rreqSentTime = millis(); // Catat waktu RREQ pertama dikirim
    pendingRREQs[slot].active = true;
    
    sendRREQ(destinationID, rreqIDCounter);
    
    Serial.printf("Route discovery started for dest=%d, rreqID=%u\n", 
                 destinationID, rreqIDCounter);
}

// Retry pending RREQs
void AODVRouting::retryRREQ() {
    unsigned long now = millis();
    
    for (int i = 0; i < 5; i++) {
        if (pendingRREQs[i].active) {
            // Check if we have route now
            if (hasRouteTo(pendingRREQs[i].destinationID)) {
                unsigned long discoveryTime = now - pendingRREQs[i].rreqSentTime;
                routeDiscoverySuccess++;

                // Emit diagnostic payload (BERHASIL via route yang sudah tersedia)
                lastDiagResult = {};
                lastDiagResult.originNodeId = myNodeID;
                lastDiagResult.targetNodeId = pendingRREQs[i].destinationID;
                if (epochOffsetPtr) {
                    lastDiagResult.rreqTimestamp = (uint32_t)(pendingRREQs[i].rreqSentTime) + *epochOffsetPtr;
                    lastDiagResult.rrepTimestamp = (uint32_t)now + *epochOffsetPtr;
                } else {
                    lastDiagResult.rreqTimestamp = (uint32_t)(pendingRREQs[i].rreqSentTime);
                    lastDiagResult.rrepTimestamp = (uint32_t)now;
                }
                lastDiagResult.discoveryMs = discoveryTime;
                lastDiagResult.hopCount = getRouteHopCount(pendingRREQs[i].destinationID);
                lastDiagResult.retryCount = pendingRREQs[i].retryCount;
                lastDiagResult.success = 1;
                hasDiagResult = true;
                if (pendingRREQs[i].destinationID < MAX_NODES) {
                    lastSuccessfulDiscoveryMs[pendingRREQs[i].destinationID] = discoveryTime;
                    lastSuccessfulDiscoveryHops[pendingRREQs[i].destinationID] = lastDiagResult.hopCount;
                }
                if (onDiagnosticReady) { onDiagnosticReady(lastDiagResult); }

                pendingRREQs[i].active = false;
                Serial.printf("Route found for dest=%d, canceling RREQ (retries=%u)\n", 
                             pendingRREQs[i].destinationID, pendingRREQs[i].retryCount);
                continue;
            }
            
            // Check timeout
            if (now - pendingRREQs[i].timestamp > RREQ_TIMEOUT) {
                if (pendingRREQs[i].retryCount < RREQ_RETRIES) {
                    pendingRREQs[i].retryCount++;
                    pendingRREQs[i].timestamp = now;
                    // Use a new RREQ ID so intermediate nodes don't drop it as duplicate.
                    rreqIDCounter++;
                    pendingRREQs[i].rreqID = rreqIDCounter;
                    sendRREQ(pendingRREQs[i].destinationID, pendingRREQs[i].rreqID);
                    Serial.printf("RREQ retry %d for dest=%d (rreqID=%u)\n", 
                                 pendingRREQs[i].retryCount, 
                                 pendingRREQs[i].destinationID,
                                 pendingRREQs[i].rreqID);
                } else {
                    pendingRREQs[i].active = false;
                    routeDiscoveryFail++;

                    // Emit diagnostic payload (GAGAL)
                    lastDiagResult = {};
                    lastDiagResult.originNodeId = myNodeID;
                    lastDiagResult.targetNodeId = pendingRREQs[i].destinationID;
                    if (epochOffsetPtr) {
                        lastDiagResult.rreqTimestamp = (uint32_t)(pendingRREQs[i].rreqSentTime) + *epochOffsetPtr;
                        lastDiagResult.rrepTimestamp = 0;
                    } else {
                        lastDiagResult.rreqTimestamp = (uint32_t)(pendingRREQs[i].rreqSentTime);
                        lastDiagResult.rrepTimestamp = 0;
                    }
                    lastDiagResult.discoveryMs = (uint32_t)(millis() - pendingRREQs[i].rreqSentTime);
                    lastDiagResult.hopCount = 0;
                    lastDiagResult.retryCount = pendingRREQs[i].retryCount;
                    lastDiagResult.success = 0;
                    hasDiagResult = true;
                    if (onDiagnosticReady) { onDiagnosticReady(lastDiagResult); }
                    Serial.printf("RREQ failed for dest=%d after %d retries | [ROUTE] OK=%d GAGAL=%d\n",
                                 pendingRREQs[i].destinationID, RREQ_RETRIES,
                                 routeDiscoverySuccess, routeDiscoveryFail);
                }
            }
        }
    }
}

// Handle RREQ packet
void AODVRouting::handleRREQ(const LoRaPacket& packet) {
    if (packet.header.payloadLength != sizeof(RREQPayload)) {
        Serial.println("Invalid RREQ payload size");
        return;
    }

    RREQPayload rreq;
    memcpy(&rreq, packet.payload, sizeof(RREQPayload));
    
    Serial.printf("RREQ received: orig=%d, dest=%d, rreqID=%u, hops=%d\n", 
                 rreq.originatorID, rreq.destinationID, rreq.rreqID, rreq.hopCount);
    
    // Check if already processed
    if (!shouldProcessRREQ(rreq.originatorID, rreq.rreqID)) {
        // SPECIAL CASE: If we are the destination, we MUST reply even if duplicate
        // This is critical for retries if the first RREP was lost
        if (rreq.destinationID == myNodeID) {
             Serial.println("Duplicate RREQ for us - Retransmitting RREP");
        } else {
             Serial.println("RREQ already processed, discarding");
             return;
        }
    }
    
    // Add to cache
    addRREQToCache(rreq.originatorID, rreq.rreqID);
    
    // Create reverse route to originator
    addRoute(rreq.originatorID, packet.header.sourceID, 
             rreq.hopCount + 1, rreq.originatorSeq);
    
    // Are we the destination?
    if (rreq.destinationID == myNodeID) {
        // Send RREP back
        incrementSequenceNumber();
        sendRREP(rreq.originatorID, myNodeID, sequenceNumber, 0);
        Serial.println("We are destination, sending RREP");
        return;
    }
    
    // Do we have fresh route to destination?
    if (hasRouteTo(rreq.destinationID)) {
        int idx = findRouteIndex(rreq.destinationID);
        if (idx < 0) {
            return;
        }
        
        // Split Horizon Rule: Don't reply if the request came from our Next Hop!
        // This prevents routing loops (e.g., 2->1->2)
        if (routingTable[idx].nextHop == packet.header.sourceID) {
             Serial.println("Loop Prevention: RREQ from next hop, ignoring.");
             return;
        }

        if (idx >= 0 && routingTable[idx].destinationSeq >= rreq.destinationSeq) {
            // Send RREP on behalf of destination
            sendRREP(rreq.originatorID, rreq.destinationID, 
                    routingTable[idx].destinationSeq, 
                    routingTable[idx].hopCount);
            Serial.println("Have fresh route, sending RREP");
            return;
        }
    }
    
    // Forward RREQ
    if (rreq.hopCount < MAX_HOP_COUNT) {
        RREQPayload forwardRREQ = rreq;
        forwardRREQ.hopCount++;
        
        LoRaPacket fwdPacket = LoRaPacketHandler::createRREQPacket(myNodeID, forwardRREQ);
        
        if (onSendPacket != nullptr) {
            onSendPacket(fwdPacket);
            Serial.println("Forwarding RREQ");
        }
    }
}

// Handle RREP packet
void AODVRouting::handleRREP(const LoRaPacket& packet) {
    // RREP is unicast hop-by-hop. Ignore packets not addressed to us.
    if (packet.header.destinationID != myNodeID &&
        packet.header.nextHop != myNodeID &&
        packet.header.destinationID != BROADCAST_ADDR) {
        return;
    }

    if (packet.header.payloadLength != sizeof(RREPPayload)) {
        Serial.println("Invalid RREP payload size");
        return;
    }

    RREPPayload rrep;
    memcpy(&rrep, packet.payload, sizeof(RREPPayload));
    
    Serial.printf("RREP received: orig=%d, dest=%d, hops=%d\n", 
                 rrep.originatorID, rrep.destinationID, rrep.hopCount);
    
    // Create route to destination
    addRoute(rrep.destinationID, packet.header.sourceID, 
             rrep.hopCount + 1, rrep.destinationSeq);
    
    // Are we the originator?
    if (rrep.originatorID == myNodeID) {
        // Hitung dan cetak waktu pembentukan rute (Route Discovery Time)
        for (int i = 0; i < 5; i++) {
            if (pendingRREQs[i].active && pendingRREQs[i].destinationID == rrep.destinationID) {
                unsigned long discoveryTime = millis() - pendingRREQs[i].rreqSentTime;
                routeDiscoverySuccess++;

                // Emit diagnostic payload
                lastDiagResult = {};
                lastDiagResult.originNodeId = myNodeID;
                lastDiagResult.targetNodeId = rrep.destinationID;
                if (epochOffsetPtr) {
                    lastDiagResult.rreqTimestamp = (uint32_t)(pendingRREQs[i].rreqSentTime) + *epochOffsetPtr;
                    lastDiagResult.rrepTimestamp = (uint32_t)millis() + *epochOffsetPtr;
                } else {
                    lastDiagResult.rreqTimestamp = (uint32_t)(pendingRREQs[i].rreqSentTime);
                    lastDiagResult.rrepTimestamp = (uint32_t)millis();
                }
                lastDiagResult.discoveryMs = discoveryTime;
                lastDiagResult.hopCount = rrep.hopCount + 1;
                lastDiagResult.retryCount = pendingRREQs[i].retryCount;
                lastDiagResult.success = 1;
                hasDiagResult = true;
                if (rrep.destinationID < MAX_NODES) {
                    lastSuccessfulDiscoveryMs[rrep.destinationID] = discoveryTime;
                    lastSuccessfulDiscoveryHops[rrep.destinationID] = rrep.hopCount + 1;
                }
                if (onDiagnosticReady) { onDiagnosticReady(lastDiagResult); }
                Serial.printf("[AODV] Route discovery time -> dest=%d: %lu ms (hops=%d) | [ROUTE] OK=%d GAGAL=%d\n",
                             rrep.destinationID, discoveryTime, rrep.hopCount + 1,
                             routeDiscoverySuccess, routeDiscoveryFail);
                break;
            }
        }
        Serial.println("RREP reached originator, route established!");
        
        // Cancel pending RREQ
        for (int i = 0; i < 5; i++) {
            if (pendingRREQs[i].active && 
                pendingRREQs[i].destinationID == rrep.destinationID) {
                pendingRREQs[i].active = false;
            }
        }
        return;
    }
    
    // Forward RREP to originator
    if (hasRouteTo(rrep.originatorID)) {
        uint8_t nextHop = getNextHop(rrep.originatorID);
        if (nextHop == BROADCAST_ADDR || nextHop == packet.header.sourceID) {
            Serial.println("Drop RREP to prevent loop");
            return;
        }

        RREPPayload forwardRREP = rrep;
        forwardRREP.hopCount++;

        incrementSequenceNumber();
        LoRaPacket fwdPacket = LoRaPacketHandler::createRREPPacket(
            myNodeID, rrep.originatorID, forwardRREP, sequenceNumber);
        fwdPacket.header.nextHop = nextHop;
        fwdPacket.header.checksum = LoRaPacketHandler::calculateChecksum(fwdPacket);
        
        if (onSendPacket != nullptr) {
            onSendPacket(fwdPacket);
            Serial.println("Forwarding RREP to " + String(nextHop));
        }
    }
}

// Handle RERR packet
void AODVRouting::handleRERR(const LoRaPacket& packet) {
    if (packet.header.payloadLength != sizeof(RERRPayload)) {
        Serial.println("Invalid RERR payload size");
        return;
    }

    RERRPayload rerr;
    memcpy(&rerr, packet.payload, sizeof(RERRPayload));
    
    Serial.printf("RERR received: unreachable node=%d\n", rerr.unreachableNodeID);
    
    // Invalidate route
    invalidateRoute(rerr.unreachableNodeID);
}

// Handle Hello message
void AODVRouting::handleHello(const LoRaPacket& packet) {
    // Hello messages help maintain neighbor discovery
    // Treat HELLO sender as 1-hop neighbor and refresh direct route.
    addRoute(packet.header.sourceID, packet.header.sourceID, 1, packet.header.sequenceNum);
    Serial.println("HELLO from Node " + String(packet.header.sourceID));
}

// Send Hello message
void AODVRouting::sendHelloMessage() {
    incrementSequenceNumber();
    LoRaPacket helloPacket = LoRaPacketHandler::createHelloPacket(myNodeID, sequenceNumber);
    
    if (onSendPacket != nullptr) {
        onSendPacket(helloPacket);
    }
}

// Send RREQ
void AODVRouting::sendRREQ(uint8_t destinationID, uint32_t rreqID) {
    incrementSequenceNumber();
    
    RREQPayload rreq;
    rreq.originatorID = myNodeID;
    rreq.destinationID = destinationID;
    rreq.rreqID = rreqID;
    rreq.originatorSeq = sequenceNumber;
    rreq.destinationSeq = 0;  // Unknown
    rreq.hopCount = 0;
    
    LoRaPacket packet = LoRaPacketHandler::createRREQPacket(myNodeID, rreq);
    
    if (onSendPacket != nullptr) {
        onSendPacket(packet);
    }
}

// Send RREP
void AODVRouting::sendRREP(uint8_t originatorID, uint8_t destinationID, 
                           uint32_t destSeq, uint8_t hopCount) {
    RREPPayload rrep;
    rrep.originatorID = originatorID;
    rrep.destinationID = destinationID;
    rrep.destinationSeq = destSeq;
    rrep.hopCount = hopCount;
    rrep.lifetime = ROUTE_TIMEOUT;
    
    uint8_t nextHop = getNextHop(originatorID);
    if (nextHop == BROADCAST_ADDR) {
        Serial.println("Cannot send RREP: no reverse route to originator");
        return;
    }
    
    incrementSequenceNumber();
    LoRaPacket packet = LoRaPacketHandler::createRREPPacket(
        myNodeID, originatorID, rrep, sequenceNumber);
    packet.header.nextHop = nextHop;
    packet.header.checksum = LoRaPacketHandler::calculateChecksum(packet);
    
    if (onSendPacket != nullptr) {
        onSendPacket(packet);
    }
}

// Send RERR
void AODVRouting::sendRERR(uint8_t unreachableNode, uint32_t unreachableSeq) {
    RERRPayload rerr;
    rerr.unreachableNodeID = unreachableNode;
    rerr.unreachableSeq = unreachableSeq;
    
    LoRaPacket packet = LoRaPacketHandler::createRERRPacket(myNodeID, rerr);
    
    if (onSendPacket != nullptr) {
        onSendPacket(packet);
    }
}

// Print routing table
void AODVRouting::printRoutingTable() {
    Serial.println("\n=== Routing Table ===");
    Serial.println("Dest\tNextHop\tHops\tSeq\tStatus");
    
    for (int i = 0; i < MAX_ROUTING_ENTRIES; i++) {
        if (routingTable[i].status != ROUTE_INVALID) {
            Serial.printf("%d\t%d\t%d\t%u\t%s\n",
                         routingTable[i].destination,
                         routingTable[i].nextHop,
                         routingTable[i].hopCount,
                         routingTable[i].destinationSeq,
                         routingTable[i].status == ROUTE_VALID ? "VALID" : "INVALID");
        }
    }
    Serial.println("====================\n");
}

// Print RREQ cache
void AODVRouting::printRREQCache() {
    Serial.println("\n=== RREQ Cache ===");
    for (int i = 0; i < MAX_ROUTING_ENTRIES; i++) {
        if (rreqCache[i].valid) {
            Serial.printf("Orig=%d, RREQ_ID=%u\n",
                         rreqCache[i].originatorID,
                         rreqCache[i].rreqID);
        }
    }
    Serial.println("==================\n");
}

// Helper: Find route index
int AODVRouting::findRouteIndex(uint8_t destination) {
    for (int i = 0; i < MAX_ROUTING_ENTRIES; i++) {
        if (routingTable[i].destination == destination && 
            routingTable[i].status != ROUTE_INVALID) {
            return i;
        }
    }
    return -1;
}

// Helper: Find free route slot
int AODVRouting::findFreeRouteSlot() {
    for (int i = 0; i < MAX_ROUTING_ENTRIES; i++) {
        if (routingTable[i].status == ROUTE_INVALID) {
            return i;
        }
    }
    return -1;
}

// Helper: Find RREQ in cache
int AODVRouting::findRREQCacheIndex(uint8_t originatorID, uint32_t rreqID) {
    for (int i = 0; i < MAX_ROUTING_ENTRIES; i++) {
        if (rreqCache[i].valid &&
            rreqCache[i].originatorID == originatorID && 
            rreqCache[i].rreqID == rreqID) {
            return i;
        }
    }
    return -1;
}

// Helper: Find free pending RREQ slot
int AODVRouting::findFreePendingSlot() {
    for (int i = 0; i < 5; i++) {
        if (!pendingRREQs[i].active) {
            return i;
        }
    }
    return -1;
}

bool AODVRouting::getLastSuccessfulDiscovery(uint8_t destination, uint32_t& discoveryMs, uint8_t& hops) {
    if (destination >= MAX_NODES) {
        return false;
    }
    if (lastSuccessfulDiscoveryHops[destination] == 0) {
        return false;
    }
    discoveryMs = lastSuccessfulDiscoveryMs[destination];
    hops = lastSuccessfulDiscoveryHops[destination];
    return true;
}

