//#pragma once
#ifndef FMSD_GUARD
#define FMSD_GUARD
#include <Arduino.h>

// which mixer to use
#define MIX_INTERNAL
#define MIX_EXTERNAL

void OpenTaps(byte drainBarrel, byte storeBarrel);
void CloseTaps(byte drainBarrel, byte storeBarrel);
void ServiceManual();
void StoppedWait();
void fmsPause(byte Source, byte Destination);
void PressureCheck(byte sens);
void FlowCheck(byte sens);
void blinkDelay(unsigned long ms, byte color);
void Fill(byte barrel, uint16_t requirement);
void Mix(byte barrel);
void Store(byte barrel, byte target);
void Drain(byte barrel, uint16_t requirement);
void Bypass();
void fmsTask(void * pvParameters);
void FillManual();
void MixManual();
void StoreManual();
void DrainManual();
void BypassManual();
void MixerExternal(bool state);

#endif