#ifndef NFC_INTEGRATION_H
#define NFC_INTEGRATION_H

#include <Arduino.h>

void nfcSubsystemSetup();
void nfcSubsystemLoop(bool wifiConnected);
bool nfcHandleSerialCommand(const String &input);
void nfcPrintStatus();
void nfcPrintHelp();
bool nfcIsBrokerConnected();

#endif
