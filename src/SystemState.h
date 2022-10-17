//#pragma once
#ifndef SYSTEM_STATE
#define SYSTEM_STATE
#include <Arduino.h>

//hex codes for "state" bit field
#define FILLING_STATE 0x10 // F = filling task on
#define MIXING_STATE 0x08  // M = mixing task on
#define STORING_STATE 0x04 // S = storing task on
#define DRAINIG_STATE 0x02 // E = draining task on
#define STOPPED_STATE 0x01 // X = stopped status on
//add sub-states here?

//hex codes for "error_state" bit-field
#define ERR_PUMP 0x01
#define ERR_WATER_PRESSURE 0x02
#define ERR_NUTRI_PRESSURE 0x04
#define ERR_WATER_NOFLOW 0x08
#define ERR_NUTRI_NOFLOW 0x10
#define ERR_NOMAINS 0x20
#define ERR_BARRELS 0x40

// Start-Stop Buttons
#define START_PIN 4
#define STOP_PIN 0

struct sSystem
{
    //bit field
    //000FMSDX
    // F = Filling state
    // M = Mixing state
    // S = Storing state
    // D = Draining state
    // X = stopped state
    byte _state_now = FILLING_STATE + STOPPED_STATE ; // 00010001 - filling + waiting for nutes - initial system state
    byte _state_before = 0;
    // bit field
    //000CBA987654321
    //0-0-no errors
    //1-1-pump error
    //2-2-water line no pressure
    //3-4-nutrients no pressure
    //4-8-no flow water
    //5-16-no flow nutrients
    //6-32-no power
    //7-64-barrels error
    //8-128-
    uint16_t _error_now = 0;
    uint16_t _error_before = 0;
    uint16_t _fill_req = 500;            // total liters to fill 
    uint16_t _mix_req = 30;               // total minutes to mix
    uint16_t _mix_timer = _mix_req;   // minutes to mix this time
    uint16_t _drain_req = 0;             // how many liters to drain
    uint32_t _drain_total = 0;                 // how many liters totally drained
    byte _filling_barrel = 0;                    // mixer barrel
    byte _storing_barrel = 0;   // will be set to last barrel in StatClass::begin 
    byte _manual_task = 0; // 0 cancel, 1 fill, 2 mix, 3 store, 4 drain, 5 bypass
    byte _manual_src = 0;
    byte _manual_dest = 0;
    uint16_t _manual_ammo = 0;
    uint16_t _bypass_req = 0;           // how many liters to fill freshwater to pools
    uint32_t _bypass_total = 0;                 // how many liters totally bypassed
};

class StatClass
{
private:
    sSystem iState;
public:
    void begin();
    bool LoadSD();
    bool SaveSD();
    byte Get();
    void IRAM_ATTR Set(byte mask);
    void IRAM_ATTR Unset(byte mask);
    void Override(byte state);
    void Apply();
    void Revert();
    bool Check(byte mask);
    void Print();
    bool isChanged();
    uint16_t Errors();
    bool isError(uint16_t error);
    void SetError(uint16_t error);
    void UnsetError(uint16_t error);
    void OverrideError(uint16_t error);
    uint16_t FillRequirement();
    void SetFillReq(uint16_t req);
    byte FillBarrel();
    void SetFillBarrel(byte barrel);
    uint16_t MixRequirement();
    void SetMixReq(uint16_t req);
    uint16_t MixTimer();
    void MixLess();
    void MixReset();
    byte StoreBarrel();
    void SetStoreBarrel(byte barrel);
    void MoveStoreUp();
    void MoveStoreDown();
    void SetDrainReq(uint16_t req);
    void SetBypassReq(uint16_t req);
    uint16_t DrainMore();
    uint16_t BypassMore();
    void DrainRecalc(byte barrel);
    void BypassRecalc();
    uint32_t DrainTotal();
    uint32_t BypassTotal();
    void SetManual(byte task, byte src, byte dest, uint16_t ammo);
    void IRAM_ATTR ResetManual();
    byte ManualTask();
    byte ManualSource();
    byte ManualDestination();
    uint16_t ManualAmmount();
};

extern StatClass State;
#endif