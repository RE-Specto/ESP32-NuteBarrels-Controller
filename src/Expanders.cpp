#include <Wire.h>
//#include "main.h"
#include "Expanders.h"
// #include "Expanders.h"
// #include "SystemState.h"
// #include "Barrels.h"
// #include "FlowSensor.h"
// #include "PresureSensor.h"
// #include "WebServer.h"
// #include "Filesystem.h"
// #include "Modem.h"
// #include "NTPClient.h"
// #include "FMSD.h"
#include "globals.h"



/*-------- Expanders Begin ----------*/
// declare I/O Expander ports and constants

// MCP23017 with pin settings
MCP23017 expander1(0); // Base Address + 0: 0x20
MCP23017 expander2(1); // Base Address + 1: 0x21

void ExpaClass::Init()
{
    // initialize TwoWire communication
    Wire.begin();

    for (size_t i = 0; i < 16; i++)
    {
        // set GPA0-7 GPB0-7 to be an output on both expanders
        expander1.getPin(i).setPinMode(OUTPUT);
        expander2.getPin(i).setPinMode(OUTPUT);
        // initialize the pins to be HIGH
        expander1.getPin(i).setValue(1);
        expander2.getPin(i).setValue(1);
    }

    // setup the MCP23017 expanders
    expander1.setup();
    expander2.setup();

    // set the init values from above
    expander1.write();
    expander2.write();

    // initial LED state
    setRGBLED(LED_WHITE);
}

//set analogMUX address
//checks bit 0-3 of mux Address "address"
//if bit set - setValue receives a positive value
//otherwise sets setValue with 0
//offset of 0 - expander 0x20 pins a0 a1 a2 a3
void ExpaClass::LockMUX(byte address)
{
    if ((_muxLock != address) && (_muxLock != MUX_UNLOCKED))
    {
        LOG.printf("MUX previously locked to %u, %u is waiting\r\n", _muxLock, address);        
    }
    while ((_muxLock != address) && (_muxLock != MUX_UNLOCKED))
        vTaskDelay(1);
    #ifdef DEBUG_MUX
    LOG.printf("MUX is free. locking to %u\r\n", address);
    #endif
    _muxLock = address;
    for (byte i = 0; i < 4; i++)
        expander1.getPin(i + 0).setValue(address & (1 << i));
    expander1.write();
}

// who locks the mux?
byte ExpaClass::GetMUX() { return _muxLock; }

// very important to run this every time you ended up business with LockMUX
void ExpaClass::UnlockMUX()
{
    if (_muxLock != MUX_UNLOCKED)
    {
        #ifdef DEBUG_MUX
        LOG.printf("MUX Unlocking %u\r\n", _muxLock);
        #endif
        _muxLock = MUX_UNLOCKED;
    }
    else
    {
        LOG.println(F("MUX already unlocked!"));
    }
}

// set RGB LED according to mask (LED_YELLOW, LED_CYAN....)
void ExpaClass::setRGBLED(byte address)
{
    for (byte i = 0; i < 3; i++)
    {
        //checks bit 0-2 of color ( R G B ) in "address"
        //offset of 4 - expander 0x20 pins a4 a5 a6
        expander1.getPin(i + 4).setValue(address & (1 << i));
    }
    expander1.write();
}

// setting this true will disable all relays
void ExpaClass::Protect(bool state)
{
    // triggers last relay in each relay board to disconnect 12v line
    expander1.getPin(15).setValue(!state); // filling relay protect pin
    expander2.getPin(7).setValue(!state);  // storing relay protect pin
    expander2.getPin(15).setValue(!state); // draining relay protect pin
    expander1.write();
    expander2.write();
    _protect = state;
    LOG.printf("!! Protection %s\r\n", state ? "On!" : "Off");
}

// returns emergency protection status
bool ExpaClass::Protect()
{
    return _protect;
}

// triggers filling relay
void ExpaClass::FillingRelay(byte address, bool state)
{
    //offset of 8 - expander 0x20 pins b0-b7
    expander1.getPin(address + 8).setValue(!state);
    expander1.write();
}

// get filling relay state
byte ExpaClass::FillingRelayGet(byte address)
{
    //offset of 8 - expander 0x20 pins b0-b7
    //expander1.read();
    return !expander1.getPin(address + 8).getValue();
}

// triggers storing relay
void ExpaClass::StoringRelay(byte address, bool state)
{
    //offset of 0 - expander 0x21 pins a0-a7
    expander2.getPin(address).setValue(!state);
    expander2.write();
}

// get storing relay state
byte ExpaClass::StoringRelayGet(byte address)
{
    //offset of 8 - expander 0x20 pins b0-b7
    //expander1.read();
    return !expander2.getPin(address).getValue();
}

// triggers draining relay
void ExpaClass::DrainingRelay(byte address, bool state)
{
    //offset of 8 - expander 0x21 pins b0-b7
    expander2.getPin(address + 8).setValue(!state);
    expander2.write();
}

// get draining relay state
byte ExpaClass::DrainingRelayGet(byte address)
{
    //offset of 8 - expander 0x20 pins b0-b7
    //expander1.read();
    return !expander2.getPin(address + 8).getValue();
}

// starts/stops the pump
void ExpaClass::Pump(bool state)
{
    expander1.getPin(14).setValue(!state); // filling relay pump pin
    expander1.write();
}
/*-------- Expanders END ----------*/
ExpaClass Expanders;
