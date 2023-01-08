//#pragma once
#ifndef FMSD_GUARD
#define FMSD_GUARD
#include <Arduino.h>

// which mixer to use
#define MIX_INTERNAL
#define MIX_EXTERNAL

#define FRESHWATER_RELAY 5
#define NUTRIENTS_RELAY 6

void ServiceManual();
void StoppedWait();
void fmsPause(byte Source, byte Destination);
void PressureCheck(byte sens);
void FlowCheck(byte sens);
void blinkDelay(unsigned long ms, byte color);
void Fill();
void Mix();
// void Store(byte barrel, byte target);
void Drain();
void Bypass();
void fmsTask(void * pvParameters);
void FillManual();
void MixManual();
void StoreManual();
void DrainManual();
void BypassManual();
void MixerExternal(bool state);

#endif