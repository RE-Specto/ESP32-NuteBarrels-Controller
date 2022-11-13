//#pragma once
#ifndef FLOW_SENSOR
#define FLOW_SENSOR
#include <Arduino.h>

void IRAM_ATTR FlowSensor1Interrupt();
void IRAM_ATTR FlowSensor2Interrupt();
void IRAM_ATTR StartButtonInterrupt();
void IRAM_ATTR StopButtonInterrupt();

//Flow sensor Pin declarations
#define FLOW_1_PIN 25
#define FLOW_2_PIN 26
//#define FLOW_3_PIN 27

struct sFlow
{
    volatile uint32_t _counter = 0;
    uint16_t _conversion_divider = 450;
    volatile uint32_t _flow = 0;
    volatile uint32_t _lastMilis = 0;
    bool _enabled = false;
    bool _need_calib = false;
};

class FSClass
{
private:
    sFlow iFsens[2]; // two sensors
public:
    void begin();
    bool LoadSD();
    bool SaveSD();
    uint16_t Get(byte sens);
    void IRAM_ATTR Count(byte sens);
    uint32_t Counted(byte sens);
    void CounterSubtract(byte sens, uint32_t value);
    void Reset(uint_fast8_t sens);
    uint16_t Divider(byte sens);
    void DividerSet(byte sens, uint16_t div);
    bool Enabled(byte sens);
    void Enable(byte sens);
    void Disable(byte sens);
    void RequestCalib(byte sens);
    void CheckNeedCalib();
    void Calib(byte sens = 1, byte liters = 5);
};

extern FSClass Flow;
#endif