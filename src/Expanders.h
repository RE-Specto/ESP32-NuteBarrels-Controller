//#pragma once
#ifndef EXPANDERS_CLASS
#define EXPANDERS_CLASS
#include <Arduino.h>
#include <MCP23017.h>

#define MUX_UNLOCKED 255 //valid channels are 0-15

// RGB_LED bitmap 00000BGR
#define LED_OFF 0x00     //000
#define LED_RED 0x01     //001
#define LED_GREEN 0x02   //010
#define LED_YELLOW 0X03  //011
#define LED_BLUE 0x04    //100
#define LED_MAGENTA 0X05 //101
#define LED_CYAN 0X06    //110
#define LED_WHITE 0X07   //111

class ExpaClass
{
private:
    byte _muxLock = MUX_UNLOCKED; // unlocked initially
    byte _protect = false;
public:
    void Init();
    void LockMUX(byte address);
    byte GetMUX();
    void UnlockMUX();
    void setRGBLED(byte address);
    void Protect(bool state);
    bool Protect();
    void FillingRelay(byte address, bool state);
    byte FillingRelayGet(byte address);
    void StoringRelay(byte address, bool state);
    byte StoringRelayGet(byte address);
    void DrainingRelay(byte address, bool state);
    byte DrainingRelayGet(byte address);
    void Pump(bool state);
};

extern ExpaClass Expanders;
#endif