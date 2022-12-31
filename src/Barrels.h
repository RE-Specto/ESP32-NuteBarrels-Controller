//#pragma once
#ifndef BARRELS_CLASS
#define BARRELS_CLASS
//#include <Arduino.h>

// load Object from Storage to Struct
#define NUM_OF_BARRELS 6
// #define POOLS 6 // barrels from 0 to 5, pools out at number 6

#define BARREL_FLUSH_ERR 0X01
#define BARREL_STORE_ERR 0X02
#define BARREL_DRAIN_ERR 0X04
#define BARREL_SONIC_CHECKSUM 0X08
#define BARREL_SONIC_TIMEOUT 0X10
#define BARREL_SONIC_OUTOFRANGE 0X20
#define BARREL_SONIC_INACCURATE 0X40
#define BARREL_DISABLED 0X80

// max allowed ultrasonic sensor deviation percents
#define SONIC_DEV 200 //5 //check disabled until more accurate sonics available

struct sBarrel
{
    //bit field - see defines above
    byte _error_state = 0;
    //float _concentraion = 0;            // percent of nutrients in solution
    //float _concentraion_last = 0;        // percent of nutrients in solution
    uint32_t _volume_freshwater = 0;     // data from flow sensor
    uint32_t _volume_freshwater_last = 0; // data from flow sensor
    uint32_t _volume_nutrients = 0;      // data from flow sensor
    uint32_t _volume_nutrients_last = 0;  // data from flow sensor
    uint16_t _sonic_last_value = 0;       // updated on every sonic measurement
    float _sonic_deviation = 0;          // +- percent error in measurement
    uint16_t _volume_min = 0; // for flow and sonic sensors - calibrate by filling and draining
    uint16_t _volume_max = 500; // for flow and sonic sensors - calibrate by filling until miscalculation
    uint16_t _ml_in_mm = 1130; // mililiters in 1 milimeter - calibrate by filling 100L, also can calculate by barrel diameter
    uint16_t _barrel_height = 1000; // mm to barrel's dry empty point - calibrate by dry barrel height
};

class BarrClass
{
private:
    sBarrel iBarrel[NUM_OF_BARRELS];
public:
    bool LoadSD();
    bool SaveSD();
    byte Errors(byte barrel);
    bool ErrorCheck(byte barrel, byte mask);
    void ErrorSet(byte barrel, byte mask);
    void ErrorUnset(byte barrel, byte mask);
    void Reset(byte barrel);
    void ErrorOverride(byte barrel, byte error);
    uint16_t FreshGet(byte barrel);
    uint16_t NutriGet(byte barrel);
    //void NutriLess(byte barrel, uint32_t liters);
    void FreshwaterFillCalc(byte barrel);
    //float ConcentrationTotal(byte barrel);
    //void ConcentrationRecalc(byte barrel);
    // bool isFillTargetReached(byte barrel, byte type, uint16_t target);
    void NutrientsTransferCalc(byte from, byte to);
    uint16_t VolumeMax(byte barrel);
    uint16_t VolumeMin(byte barrel);
    void VolumeMaxSet(byte barrel, uint16_t volume);
    void VolumeMinSet(byte barrel, uint16_t volume);
    void SonicMeasure(byte barrel, byte measure = 10, uint16_t timeLeft = 1000, byte retryLeft = 5);
    uint16_t SonicLastMM(byte barrel);
    int16_t SonicLastMMfromEmpty(byte barrel);
    uint16_t SonicOffset(byte barrel);
    uint16_t SonicMLinMMGet(byte barrel);
    void SonicOffsetSet(byte barrel, uint16_t offs);
    void SonicMLinMMSet(byte barrel, uint16_t coef);
    int16_t SonicCalcLiters(byte barrel);
    void Save100LitersMark(byte barrel);
    int8_t BarrelPercents(byte barrel);
    int16_t SonicLitersTotal();
    int16_t SonicLitersUsable();
    bool isDry(byte barrel);
    bool isEmpty(byte barrel);
    bool isFull(byte barrel);
    void TestSensors();
};

extern BarrClass Barrels;
#endif