//#pragma once
#ifndef PRESSURE_SENSOR
#define PRESSURE_SENSOR
#include <Arduino.h>

// _sensorPin Pressure sensor Pin declarations 
#define PRESSUR_SENSORS 2
#define PRESSUR_1_PIN 36
#define PRESSUR_2_PIN 34
//#define PRESSUR_3_PIN 35

// _error_state pressure sensor state codes
#define PRESSURE_NORMAL 0
#define PRESSURE_NOPRESSURE 1
#define PRESSURE_OVERPRESSURE 2
#define PRESSURE_DISCONNECT 3
#define PRESSURE_SHORTCIRCUIT 4

struct sPressure
{
    byte _error_state = 0;
    byte _sensorPin = 255; // defaults
    byte _divider = 10;    //36;
    int16_t _offset = 0;      //-145;
    byte _max_pressure = 200;
    byte _min_pressure = 55;
    int16_t _last_pressure = 0;
    bool _enabled = false;
};

class PSClass
{
private:
    sPressure iPsens[PRESSUR_SENSORS]; // two sensor
public:
    void begin();
    bool LoadSD();
    bool SaveSD();
    void setSensor(byte sens, byte sensorPin);
    byte Divider(byte sens);
    void DividerSet(byte sens, byte div);
    int16_t Offset(byte sens);
    void OffsetSet(byte sens, int16_t offs);
    byte Max(byte sens);
    void MaxSet(byte sens, byte max);
    byte Min(byte sens);
    void MinSet(byte sens, byte min);
    int16_t LastValue(byte sens);
    int16_t measure(byte sens);
    byte Errors(byte sens);
    void Reset(byte sens);
    bool Enabled(byte sens);
    void Enable(byte sens);
    void Disable(byte sens);
};

extern PSClass Pressure;
#endif