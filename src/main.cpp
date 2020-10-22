/*
bugs:
system resets every time serial console connects/disconnects?
    https://esp32.com/viewtopic.php?t=4988
void store allways storing untill empty
    must give option to store only required ammount
if mix stopped in the middle - it will count from start again
pressure offset is redundant?
[1126][measure] measuring pSensor 1 @pin36 calib:0 value:[1129][measure] 0
[1280][ErrorSet] [E] Barr0:e16 should report sms?

to implement:

an option to manually start fill mix store drain from webUI
    ---[mix]Status Stopped and not Manual. breaking.
        add "set manual mode on off" from /manual
    add a cancel button
        will webui hangs during long execution?
        webserver should run in a separate thread?
            manual commands can leave a flag for fmsTask to pick up
                check line 2000
fmsd functions should return right away without starting if stopped and not manual?

test:
    fill huge ammount
    mix huge ammount
    fill 0
    fill 6
    store to itself
    store from 0 to 6
    store from 6 to 0
    drain 0
    drain 6

barr error set shoud save to sd? or save after each use in code?
    implement error unset from webui

serial print to telnet or to webui directly or via another board (esp8266?).

-solder start-stop rgb led test cable
-solder dummy pressure sensor

add a way to reset pressure error "protect" mode - by pressing start?
use combination of SystemState.state_check and Transfers.inner_state
    on reset - sytem will remeasure using sonic 
        if diff too large - assign sonic value to 
            VolumeNutrients or VolumeFreshwater depending on last substate
            if sub-state was nutri transfer - recalc concentration?
                make concentration calc function universal?
    system will continue from last substate

!! need debaunce!! - once pressed loop check if high 50 ms, check if still pressed?
    from within receiving functions only?
        interrupts only sets variables?
             each task checks for this flag, loop-measure for 50ms
                break loop if released
                acts accordingly otherwise
                https://www.evernote.com/shard/s544/nl/96519974/8b3f9bc9-551e-4f13-b3cd-98f8cc75c980

start/stop interrupts "use start-stop actions from evernote!" 
    start sets canMix global boolean? 
    if stopped also pressed - do something else
    if status = stopped: clear stopped. else if no status: set mixing. else do start-button extra action
        task or loop reads the bool - changes RGB immediately, but start mixing task only when no tasks running
    stop sets shouldStop global boolean?
    if start also pressed - do something else
    if status not stopped - set stopped + save. else save + extra stopped action
        task or loop reads the bool - changes RGB immediately, triggers stop and save
    on stop:     pressure.SaveSD(); flow.SaveSD();
    // can set the bool to !bool to check if pressed twice - for reset and other system tasks

add setRGBLED to functions

work on system hardware - implement all changes to schematic!!
change zeners to 3.3v? - check new zeners that arrived

check what modem returns if error - if valid check - implement SMS error

add calibration edit for all sensors to manual: 
use separate <forms> for each one?
    SystemState.state_set
    SystemState.state_unset
    SystemState.error_set
    SystemState.error_unset
    flow.MultSet(sens,mult)
    pressure.DividerSet
    pressure.OffsetSet
    pressure.MaxSet
    pressure.MinSet
    barrels.ErrorSet
    barrels.ErrorUnset
    barrels.ErrorReset
    barrels.SonicOffsetSet
    barrels.SonicMLinMMSet
    Transfers.prefill_requirement
    Transfers.afterfill_requirement deprecate??
    Transfers.mix_requirement
    Transfers.drain_requirement
    Transfers.filling_barrel
    Transfers.storing_barrel
    NTP Timezone + DTS

manual apply sonic value to flow barrel volume on request.

WebUI from "1-pump idea test (1).png" file with overlays and jQuery
    https://forum.jquery.com/topic/how-to-change-text-dynamically-svg
telnet server to send LOG.prints + telnet client inside webui?
allow to manually disable a barrel
    barrels.ErrorSet(barrel, BARREL_DISABLED)

fmsTask - check fill mix if barrel 0 not error!? 
    who can set barrel 0 error?

add LOG.println(__FUNCTION__); at every function start for debug
    add uptime?
    typeid(*this).name() for class name?
        __FUNCTION__ is non standard, __func__ exists in C99 / C++11. The others (__LINE__ and __FILE__) are just fine.
        It will always report the right file and line (and function if you choose to use __FUNCTION__/__func__). Optimization is a non-factor since it is a compile time macro expansion; it will never effect performance in any way.
------------------------------

uint16_t DTS = 0;                                              //mySettings.DTS?3600UL:0;
uint16_t timeZone = 3;    

limit all data from webUI to valid range - ie. no "measure barrel 27"

variables that survives reboot?? :)
try RTC_NOINIT_ATTR?
https://github.com/highno/rtcvars/issues/3

recheck concentration float is accurate
"Do not use float to represent whole numbers."
http://www.cplusplus.com/forum/general/67783/
    check my possible ranges:
    test on another esp32
concentration float check if ok.

make pressure sensor check constantly so i can turn off the pump on overpressure
    task loop inside other cpu core? or inside filling loop itself?
    ps2 only when pump working?
    ps1 only once before sol open + once if no flow + once at system start?

reset all sensor values on system start? flow only? or remeber who to assing to and assign on start?
    is it important at all? how much water can flow between assignments?

deprecate RTC? replace with ntp + timeAlarms?

set Overpressure global error only if error occurs with multiple solenoids

check all uint values never go below zero!!
    expected in sensor measurements
    anywhere else?

implement mux lock timeout.

measure all sonic on system start so sonic have all values ready?

change sonic 10555  to retry--
    BARREL_SONIC_OUTOFRANGE only if all 10555

handle reset in the middle of flow sensor transfer
    calculate by last values?
    lock thread mux so it executes quick? https://esp32.com/viewtopic.php?t=1703

deprecate error reporting task and sendsms dictionary?

add to schematic:
12v to 5v change to switching regulator? move power supply out of the main board?
12v line add polyfuze
optional: doser on expander x20 pin A7? serial scale mux#14?


optional:
add concentrationSet to barrels for manual?
use filter instead of avearge for sonic?
https://stackoverflow.com/questions/10338888/fast-median-filter-in-c-c-for-uint16-2d-array
restore to sd if spiffs date newer? fix timestamp
barrel busy flag?
add option to continue on pressure sensor1 error if flow1 ok
continue on ps2 error if flow2 is in range: 0 < flow2 < "flow2 with no load"
continue on flow error if sonic is rising
continue on sonic error if flow counted barrel volume is ok
continue to next barrel if barrel solenoids error
only unrecoverable errors: pump, main barrel solenoids
add start-stop from webUI

server.on("/mdns" check mdns broadcast
switchFS add fs check, try to restart filesystem on failure
add ... on serial.available to check module awake
"[E][vfs_api.cpp:22] open(): File system is not mounted" nice error reporting format 
*/
#include <Arduino.h>
#include <Wire.h>
#include <MCP23017.h>
#include <SD.h>
#include "SPIFFS.h"
//#include <RtcDS3231.h>
#include <WiFi.h>

//needed for library
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> //https://github.com/tzapu/WiFiManager

//#include <Hash.h>
#include <AsyncTCP.h>
#include <ArduinoOTA.h>
#include "ESPmDNS.h"
//https://github.com/jandrassy/TelnetStream
#include "TimeAlarms.h"

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//hex codes for "error_state" bit-field
#define ERR_PUMP 0x01
#define ERR_WATER_PRESSURE 0x02
#define ERR_NUTRI_PRESSURE 0x04
#define ERR_WATER_NOFLOW 0x08
#define ERR_NUTRI_NOFLOW 0x10
#define ERR_NOMAINS 0x20
#define ERR_BARRELS 0x40

//hex codes for "state" bit field
#define MANUAL_STATE 0x20  // manual mode on
#define FILLING_STATE 0X10 // F = filling task on
#define MIXING_STATE 0x08  // M = mixing task on
#define STORING_STATE 0x04 // S = storing task on
#define DRAINIG_STATE 0x02 // E = draining task on
#define STOPPED_STATE 0x01 // X = stopped status on
//add sub-states here?

// RGB_LED bitmap 00000BGR
#define LED_OFF 0x00     //000
#define LED_RED 0x01     //001
#define LED_GREEN 0x02   //010
#define LED_YELLOW 0X03  //011
#define LED_BLUE 0x04    //100
#define LED_MAGENTA 0X05 //101
#define LED_CYAN 0X06    //110
#define LED_WHITE 0X07   //111

#define BARREL_FLUSH_ERR 0X01
#define BARREL_STORE_ERR 0X02
#define BARREL_DRAIN_ERR 0X04
#define BARREL_SONIC_CHECKSUM 0X08
#define BARREL_SONIC_TIMEOUT 0X10
#define BARREL_SONIC_OUTOFRANGE 0X20
#define BARREL_SONIC_INACCURATE 0X40
#define BARREL_DISABLED 0X80

// pressure sensor state codes
#define PRESSURE_NORMAL 0
#define PRESSURE_NOPRESSURE 1
#define PRESSURE_OVERPRESSURE 2
#define PRESSURE_DISCONNECT 3
#define PRESSURE_SHORTCIRCUIT 4

#define PRES_SENSOR_FRESHWATER 1
#define PRES_SENSOR_NUTRIENTS 2

#define FLOW_SENSOR_FRESHWATER 1
#define FLOW_SENSOR_NUTRIENTS 2

#define BARREL_FRESHWATER 1
#define BARREL_NUTRIENTS 2


#define MUX_UNLOCKED 255 //valid shannels are 0-15

#define LOG Serial.printf("[%04i][%s] ", __LINE__, __FUNCTION__);Serial   //SerialAndTelnet or file
#define SYNC_INTERVAL 600 // NTP sync - in seconds
#define NTP_PACKET_SIZE 48
// NTP time is in the first 48 bytes of message
#define NTP_HOSTNAME "pool.ntp.org"
#define NTP_PORT 123
#define NTP_TIMEOUT 5000
#define LOCAL_UDP_PORT 8888 // local port to listen for UDP packets

// load Object from Storage to Struct
#define NUM_OF_BARRELS 6

//Flow sensor Pin declarations
#define FLOW_1_PIN 25
#define FLOW_2_PIN 26
//#define FLOW_3_PIN 27

//Pressure sensor Pin declarations
#define PRESSUR_1_PIN 36
#define PRESSUR_2_PIN 34
//#define PRESSUR_3_PIN 35

// delay for SMS error report
#define REPORT_DELAY 5000

#define DEBUG


// Globals
FS *disk = &SPIFFS; // default
File file;
WiFiUDP UDP;
AsyncWebServer server(80);
DNSServer dns;
bool isTimeSync = false;
bool isSaving = false;
bool isSD = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // for interrupts and xTasks
char smsMessage[64];                             // filled by MessageFromDict from dictionary

//function declarations:
bool Load(const char *fname, byte *stru_p, uint16_t len);
bool Save(const char *fname, byte *stru_p, uint16_t len);
void SaveStructs();
void IRAM_ATTR FlowSensor1Interrupt();
void IRAM_ATTR FlowSensor2Interrupt();
void SendSMS(const char *message, byte item = 0xFF);

/*-------- Filesystem code ----------*/
// SD Card and SPIFFS
void initStorage()
{
    byte retry = 5; // 5 times for SD
    while (retry)
    {
        if (SD.begin())
        {
            LOG.println(F("SD card OK"));
            disk = &SD;
            isSD = true;
            break;
        }
        else
        {
            LOG.println(F("Error: Failed to initialize SD card"));
            Alarm.delay(1000);
            retry--; // prevent dead loop
        }
    }
    if (!isSD)
        SendSMS("SD Card Error - please check");
    retry = 3; // 3 times for SPIFFS
    while (retry)
    {
        if (SPIFFS.begin())
        {
            LOG.println(F("SPIFFS OK"));
            break;
        }
        else
        {
            LOG.println(F("Error: Failed to initialize SPIFFS"));
            Alarm.delay(1000);
            retry--; // prevent dead loop
        }
    }
}

// loading Structs from files
// filename, struct pointer, struct lenght "sizeof(myStruct)"
bool Load(const char *fname, byte *stru_p, uint16_t len)
{
    uint16_t count = 0;
    LOG.printf("Loading %s\t", fname);
    file = disk->open(fname, "r");
    if (!file)
    {
        LOG.print(F("unable to open file\r\n\r\n"));
    }
    else
        for (; count < len; count++)
            if (file.available())
                *(stru_p + count) = file.read();
#ifdef DEBUG
    LOG.printf("%s\r\n%u out of %u Bytes read. filesize %satch.\r\n", count == len ? "successfully" : "failed", count, len, len == file.size() ? "M" : "Mism");
#else
    LOG.println();
#endif
    file.close();
    return count == len;
}

// save structs to files
bool Save(const char *fname, byte *stru_p, uint16_t len)
{
    uint16_t count = 0;
    LOG.printf("Saving %s\t", fname);
    file = disk->open(fname, "w"); // creates the file of not exist
    //file.setTimeCallback(timeCallback);
    if (!file)
    {
        LOG.print(F("unable to open file\r\n\r\n"));
    }
    else
        count = file.write(stru_p, len); // save Logic
#ifdef DEBUG
    LOG.printf("%s\r\n%u out of %u Bytes writen. filesize %satch.\r\n", count == len ? "successfully" : "failed", count, len, len == file.size() ? "M" : "Mism");
#else
    LOG.println();
#endif
    file.close();
    return count == len;
}

// backup overwrites all files on SPIFFS
byte Backup()
{
    LOG.println(F("Backing up all files from SD to SPIFFS"));
    File dir = SD.open("/");
    File file = dir.openNextFile();
    size_t len = 0; // file chunk lenght at the buffer
    byte counter = 0;
    while (file)
    {
        static byte buf[512];
        if (!file.size())
        {
            LOG.printf("skipping empty file %s\r\n", file.name());
        }
        else
        {
            // was unable to detect SD disconnection - file was true, name and size returned valid, only read returned zero.
            if (!file.read(buf, 512))
            {
                LOG.println(F("[E] SD not exist!!"));
                break;
            }             // protect against endless loop on SD error
            file.seek(0); // start from start
            File destFile = SPIFFS.open(file.name(), FILE_WRITE);
            while (file.available())
            {
                len = file.read(buf, 512);
                //stream.readBytes(buffer, length)
                destFile.write(buf, len);
                LOG.printf("copying %s from SD to SPIFFS %u bytes copied\r\n", destFile.name(), len);
            }
            destFile.close();
            counter++;
        }
        file.close();
        file = dir.openNextFile();
    }
    dir.close();
    LOG.printf("Backup finished. %u files copied\r\n", counter);
    return counter;
}

// restore only files missing on SD card
byte Restore()
{
    LOG.println(F("Restoring missing files from SPIFFS to SD"));
    File dir = SPIFFS.open("/");
    File file = dir.openNextFile();
    size_t len = 0; // file chunk lenght at the buffer
    byte counter = 0;
    while (file)
    {
        //file.name() file.size());
        if (SD.exists(file.name()))
        {
            LOG.printf("file %s already exist on SD\r\n", file.name());
        }
        else if (!file.size())
        {
            LOG.printf("skipping empty file %s\r\n", file.name());
        }
        else
        {
            File destFile = SD.open(file.name(), FILE_WRITE);
            LOG.println(file.name());
            if (destFile)
            {
                static byte buf[512];
                //memset(buf, 0, 512); // zerofill the buffer
                while (file.available())
                {
                    len = file.read(buf, 512);
                    destFile.write(buf, len);
                    LOG.printf("copying %s from SPIFFS to SD %u bytes copied\r\n", destFile.name(), len);
                }
            }
            else
            {
                LOG.println(F("Error writing to SD"));
            }
            destFile.close();
            if (file.size() != destFile.size())
            {
                LOG.printf("[E] file %s size mismatch!\r\n", file.name());
            }
            else
                counter++;
        }
        file.close();
        file = dir.openNextFile();
    }
    dir.close();
    LOG.printf("Restore finished. %u files copied\r\n", counter);
    return counter;
}
/*-------- Filesystem END ----------*/

// declare I/O Expander ports and constants

// MCP23017 with pin settings
MCP23017 expander1(0); // Base Address + 0: 0x20
MCP23017 expander2(1); // Base Address + 1: 0x21

class exp
{
private:
    byte _muxLock = MUX_UNLOCKED; // unlocked

public:
    void Init()
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

        // initial LED state
        setRGBLED(LED_WHITE);
    }

    //set analogMUX address
    //checks bit 0-3 of mux Address "address"
    //if bit set - setValue receives a positive value
    //otherwise sets setValue with 0
    //offset of 0 - expander 0x20 pins a0 a1 a2 a3
    void setMUX(byte address)
    {

        if ((_muxLock != address) && (_muxLock != MUX_UNLOCKED))
        {
            LOG.printf("MUX previously locked to %u, %u is waiting\r\n", _muxLock, address);
        }

        while ((_muxLock != address) && (_muxLock != MUX_UNLOCKED))
            Alarm.delay(1);

        LOG.printf("MUX is free. locking to %u\r\n", address);
        _muxLock = address;

        for (byte i = 0; i < 4; i++)
            expander1.getPin(i + 0).setValue(address & (1 << i));

        expander1.write();
    }

    // who locks the mux?
    byte GetMUX() { return _muxLock; }

    // very important to run this every time you ended up business with setMUX
    void UnlockMUX()
    {
        if (_muxLock != MUX_UNLOCKED)
        {
            LOG.printf("MUX Unlocking %u\r\n", _muxLock);
            _muxLock = MUX_UNLOCKED;
        }
        else
        {
            LOG.println(F("MUX already unlocked!"));
        }
    }

    // set RGB LED according to mask (LED_YELLOW, LED_CYAN....)
    void setRGBLED(byte address)
    {
        for (byte i = 0; i < 3; i++)
        {
            //checks bit 0-2 of color ( R G B ) in "address"
            //offset of 4 - expander 0x20 pins a4 a5 a6
            expander1.getPin(i + 4).setValue(address & (1 << i));
        }
        expander1.write();
    }

    // setting true will disable all relays
    void Protect(bool state)
    {
        // triggers last relay in each relay board to disconnect 12v line
        expander1.getPin(15).setValue(!state); // filling relay protect pin
        expander2.getPin(7).setValue(!state);  // storing relay protect pin
        expander2.getPin(15).setValue(!state); // draining relay protect pin
        expander1.write();
        expander2.write();
    }

    // triggers filling relay
    void FillingRelay(byte address, bool state)
    {
        //offset of 8 - expander 0x20 pins b0-b7
        expander1.getPin(address + 8).setValue(!state);
        expander1.write();
    }

    // get filling relay state
    byte FillingRelayGet(byte address)
    {
        //offset of 8 - expander 0x20 pins b0-b7
        //expander1.read();
        return !expander1.getPin(address + 8).getValue();
    }

    // triggers storing relay
    void StoringRelay(byte address, bool state)
    {
        //offset of 0 - expander 0x21 pins a0-a7
        expander2.getPin(address).setValue(!state);
        expander2.write();
    }

    // get storing relay state
    byte StoringRelayGet(byte address)
    {
        //offset of 8 - expander 0x20 pins b0-b7
        //expander1.read();
        return !expander2.getPin(address).getValue();
    }

    // triggers draining relay
    void DrainingRelay(byte address, bool state)
    {
        //offset of 8 - expander 0x21 pins b0-b7
        expander2.getPin(address + 8).setValue(!state);
        expander2.write();
    }

    // get draining relay state
    byte DrainingRelayGet(byte address)
    {
        //offset of 8 - expander 0x20 pins b0-b7
        //expander1.read();
        return !expander2.getPin(address + 8).getValue();
    }

    // starts/stops the pump
    void Pump(bool state)
    {
        expander1.getPin(14).setValue(!state); // filling relay pump pin
        expander1.write();
    }

} expanders; // initiated globally

// message... item number (optional)
void SendSMS(const char *message, byte item)
{ //byte item=0xFF moved to declaration BOF
    if (item != 0xFF)
    {
        LOG.printf("sms: %s %u\r\n", message, item);
    }
    else
    {
        LOG.printf("sms: %s\r\n", message);
    }

    expanders.setMUX(7);                          // modem is at port 7
    Alarm.delay(10);                              // wait until expander + mux did their job
    Serial2.println("AT+CMGS=\"+972524373724\""); //change ZZ with country code and xxxxxxxxxxx with phone number to sms
    Serial2.print(message);                       //text content
    if (item != 0xFF)
        Serial2.print(item);
    Serial2.write(26); // send ctrl+z end of message
    //Serial2.print((char)26); // if the above won't work - thy this one instead
    expanders.UnlockMUX();
    Serial.println(); // add newline after sendsms for log readability
}

/*
module needs 3.4V to 4.4V (Ideal 4.1V) @ 2amp
mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
mySerial.println("AT+CREG?"); //Check whether it has registered in the network
ATI – Get the module name and revision
AT+COPS? – Check that you’re connected to the network, in this case BSNL
AT+COPS=? – Return the list of operators present in the network.
AT+CBC – will return the lipo battery state. The second number is the % full (in this case its 93%) and the third number is the actual voltage in mV (in this case, 3.877 V)
*/

// !!! deprecate dict message below???
/*
//https://forum.arduino.cc/index.php?topic=451141.0
// search file dictX, skip error_number commas, set value untill next comma to smsMessage[]
// seaprate dictionary for each caller_function
void MessageFromDict(byte caller_function, byte error_number){
    // dictionary files:
    // system
    // barrels
    // fmsd tasks
    // time?
    // filesystem?

    char filename[8];
    sprintf (filename, "dict%u", caller_function);
    // smsMessage[64] to fill.
}

void SendSMS(byte caller_function, byte error_number, byte item_number=0xFF){
    Serial.printf("[SendSMS] [caller_function:%u] [error_number:%u] [item_number:%u]\r\n", 
        caller_function, error_number, item_number);
    char buff[96]; // 64 message + else
    // use dictionary from file - comma separated
    MessageFromDict(caller_function, error_number);
    if (item_number==0xFF) {
        Serial.println(smsMessage);
        SendSMS(smsMessage);
    }
    else {
        sprintf (buff, "%s: %u", smsMessage, item_number);
        Serial.println(buff);
        SendSMS(buff);
    }
    //error from dict - item show as is - default item 255 - if default item - no item.
    //[system] [error: overpressure sensor:] [1]
    //[barrels] [warning: barrel disabled:] [4]
}

// set MUX to SIM_MUX_ADDRESS
// send sms with error code description
void SendSMS(String message){
    LOG.print("-sendsms: ");
    LOG.println(message);
    SendSMS(message.c_str());
}*/

void modemInit()
{
    LOG.println("-modem init");
    expanders.setMUX(7); // modem is at port 7
    Alarm.delay(10);     // wait until expander + mux did their job
    // Serial2.begin(9600, SERIAL_8N1); // already done in main
    Serial2.println("AT");        //Once the handshake test is successful, it will back to OK
    Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
    expanders.UnlockMUX();        // Must unlock after every use!!
}

struct st
{
    uint16_t prefill_requirement = 100;   // initial 100 liters
    uint16_t afterfill_requirement = 500; // additional 400 liters - total 500L
    uint16_t mix_requirement = 30;        // minutes to mix
    // transfer requirement calculated dynamically by barrel ammount
    // transfer counter applied dynamically at transfer from source to destination
    uint16_t drain_requirement = 0;              //in liters - how much to drain
    uint32_t drain_counter = 0;                  //in liters - how much totally drained
    byte filling_barrel = 0;                  // mixer barrel
    byte storing_barrel = NUM_OF_BARRELS - 1; // last barrel (-1 cause we start from zero)
} Transfers;

struct myST
{
    //bit field
    //00NFMSEX
    // N = maNual mode on - overrides stopped
    // F = Filling task on
    // M = Mixing task on
    // S = Storing task on
    // E = Emptying task on - overrides f,m,s
    // X = stopped status on
    // 00NFMSEX
    byte _state_now = 17; // 00010001 - filling + waiting for nutes - initial system state
    byte _state_before = 0;

    // bit field
    //000CBA987654321
    //0-0-no errors
    //1-1-water line no pressure
    //2-2-pump error
    //3-4-
    //4-8-
    //5-16-
    //6-32-
    //7-64-
    //8-128-
    //9-256-
    //A-512-
    //B-1024-
    //C-2048-
    //D-4096-
    //E-8192-
    //F-16384-
    uint16_t _error_now = 0;
    uint16_t _error_before = 0;
};

class STClass
{
private:
    myST myState;

public:
    bool LoadSD() { return Load("/SysState.bin", (byte *)&myState, sizeof(myState)); }

    bool SaveSD() { return Save("/SysState.bin", (byte *)&myState, sizeof(myState)); }

    // returns system state raw integer value (0=uninitiated)
    byte state_get() { return myState._state_now; }

    // set state using mask (FILLING_STALE,STORING_STATE....)
    void state_set(byte mask) { myState._state_now |= mask; }

    // unset mask from state  (FILLING_STALE,STORING_STATE....)
    void state_unset(uint16_t mask) { myState._state_now &= ~mask; }

    // preserve prevoius state
    void state_save() { myState._state_before = myState._state_now; }

    // restore previous state
    void state_load() { myState._state_now = myState._state_before; }

    // returns true if state have "mask-bit" state on. ex: return_state(MIXING_STATE);
    bool state_check(byte mask) { return myState._state_now & mask; }

    // returns error state
    uint16_t error_get() { return myState._error_now; }

    // set error using mask
    void error_set(uint16_t error) { myState._error_now |= error; }

    // unset error using mask
    void error_unset(uint16_t error) { myState._error_now &= ~error; }

    //returns the difference between current and last error states
    // !! need to reimplement to return either the new error position
    // so I dont have to use uint16_t for return
    // or either if two errors should be reported at once - a while loop that solves them one by one
    uint16_t error_getnew()
    {
        // bitwise - diff between "before" and "now",
        // compare with "now" to extract new bits only
        return (myState._error_now ^ myState._error_before) & myState._error_now;
    }

    void error_reported() { myState._error_before = myState._error_now; }

} SystemState;

// deprecate?? send sms on demand in each function?
//what if need to send sms in middle of mux lock? sonic measurement?
// Task 0 - Error reporting task
void errorReportTask()
{
    // while there is no new errors
    while (!SystemState.error_getnew())
    {
        Alarm.delay(REPORT_DELAY); //wait - endless loop untill error status changes
    }

    // goes here if there is an error to report
    uint16_t newerrors = SystemState.error_getnew();
    uint16_t counter = 1;
    while (newerrors)
    { // loops untill error is empty
        if (newerrors & counter)
        { // try each error state bit one by one
            LOG.printf("system error %u\r\n", counter);
            //SendSMS( SYSTEM_ERROR, counter );
            newerrors -= counter; // Subtract what already reported
            counter *= 2;         // next bit
        }
    }
    // clear error after all being reported
    SystemState.error_reported();
}

// flow Sensors Pin Declarations
// and Interrupt routines

struct myFSENS
{
    volatile uint32_t counter = 0;
    uint16_t conversion_divider = 450;
    volatile uint32_t flow = 0;
    volatile uint32_t lastMilis = 0;
};

/*
Pulses per Liter: 450
Model: YF-S201

From the spec sheet:
16 Hz = 2 L/min = 120 L/hr 
32.5 Hz = 4 L/min = 240 L/hr
49.3 Hz = 6 L/min = 360 L/hr
65.5 Hz = 8 L/min = 480 L/hr
82 Hz = 10 L/min = 600 L/hr
etc.  Max 30 L/min
*/

class FSClass
{
private:
    myFSENS fsensor[2]; // two sensors
public:
    // returns flow in mililiters per second for sensor No(sens)
    uint16_t FlowGet(byte sens)
    {
        // sensors now start from 1 (but arrays from 0)
        sens--;
        // measurement older than 1 second means no flow
        if ((millis() - fsensor[sens].lastMilis) > 1000)
            return 0;
        return ((1000000 / (fsensor[sens].flow ? fsensor[sens].flow : 1)) / fsensor[sens].conversion_divider); // in mililiters per second
    }                                                                                                          // Guru Meditation Error: Core  1 panic'ed (IntegerDivideByZero). Exception was unhandled.

    // executed by interrupt to increase counter
    void IRAM_ATTR CounterInc(byte sens)
    {
        fsensor[sens].counter++;
        fsensor[sens].flow = millis() - fsensor[sens].lastMilis; // interval in miliseconds from the last call
        fsensor[sens].lastMilis = millis();
    }

    // get raw flow counter pulses
    uint32_t CounterGet(byte sens) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        return fsensor[sens].counter; 
    }

    // Subtracts count from sensor (after we applied the count to target barrel)
    void CounterSubtract(byte sens, uint32_t value)
    {
        // sensors now start from 1 (but arrays from 0)
        sens--;
        if (fsensor[sens].counter >= value)
            fsensor[sens].counter -= value;
        else
        {
            fsensor[sens].counter = 0;
            LOG.printf("Err: tried to decrease flowsensor%u below zero, with %u\r\n", sens + 1, value);
        }
    }

    // reset count for sensor (sens)
    void CounterReset(uint_fast8_t sens) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        fsensor[sens].counter = 0; 
    }

    // returns pulses to liter
    uint16_t DividerGet(byte sens) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        return fsensor[sens].conversion_divider; 
    }

    // set pulses to liter (sensor number, divider)
    void DividerSet(byte sens, uint16_t div) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        fsensor[sens].conversion_divider = div; 
    }

    bool LoadSD() { return Load("/Flow.bin", (byte *)&fsensor, sizeof(fsensor)); }

    bool SaveSD() { return Save("/Flow.bin", (byte *)&fsensor, sizeof(fsensor)); }

    // attach interrupts
    void begin()
    {
        LOG.printf("-Flow: init sensors at pins %u, %u\r\n", FLOW_1_PIN, FLOW_2_PIN);
        pinMode(FLOW_1_PIN, INPUT_PULLUP);
        pinMode(FLOW_2_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(FLOW_1_PIN), FlowSensor1Interrupt, FALLING);
        attachInterrupt(digitalPinToInterrupt(FLOW_2_PIN), FlowSensor2Interrupt, FALLING);
    }

} flow;

//https://esp32.com/viewtopic.php?t=1703
//flow sensor 1 interrupt routine
void IRAM_ATTR FlowSensor1Interrupt()
{
    portENTER_CRITICAL_ISR(&mux);
    flow.CounterInc(0);
    portEXIT_CRITICAL_ISR(&mux);
}
//flow sensor 2 interrupt routine
void IRAM_ATTR FlowSensor2Interrupt()
{
    portENTER_CRITICAL_ISR(&mux);
    flow.CounterInc(1);
    portEXIT_CRITICAL_ISR(&mux);
}

// Pressure Sensors Pin Declarations
// and Analog reads
// task pressure sensors - stops pumps on overpressure

struct myPS
{
    //0 normal pressure
    //1 no pressure
    //3 overpressure
    //2 disconnected
    //4 short circuit

    byte _ErrorState = 0;
    byte _sensorPin = 255; // defaults
    byte _divider = 10;    //36;
    int16_t _offset = 0;      //-145;
    byte _max_pressure = 200;
    byte _min_pressure = 55;
    int16_t _last_pressure = 0;
};

// pressure sensors starts from 0
class pressure_sensor
{
private:
    myPS psensor[2]; // two sensor
public:
    // init the sensor
    void setSensor(byte sens, byte sensorPin)
    {
        // sensors now start from 1 (but arrays from 0)
        sens--;
        psensor[sens]._sensorPin = sensorPin;
        pinMode(sensorPin, INPUT); // initialize analog pin for the sensor
    }

    byte DividerGet(byte sens) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        return psensor[sens]._divider; 
    }

    void DividerSet(byte sens, byte div) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        psensor[sens]._divider = div; 
    }

    int16_t OffsetGet(byte sens) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        return psensor[sens]._offset; 
    }

    void OffsetSet(byte sens, int16_t offs) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        psensor[sens]._offset = offs; 
    }

    byte MaxGet(byte sens) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        return psensor[sens]._max_pressure; 
    }

    void MaxSet(byte sens, byte max) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        psensor[sens]._max_pressure = max; 
    }

    byte MinGet(byte sens) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        return psensor[sens]._min_pressure; 
    }

    void MinSet(byte sens, byte min) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        psensor[sens]._min_pressure = min; 
    }

    bool LoadSD() { return Load("/Pressure.bin", (byte *)&psensor, sizeof(psensor)); }

    bool SaveSD() { return Save("/Pressure.bin", (byte *)&psensor, sizeof(psensor)); }

    // returns last value measured by pressure sensor (sens)
    int16_t LastValue(byte sens)
    {
        return psensor[sens]._last_pressure;
    }

    // read sensor (sens) - converts analog value to pressure
    int16_t measure(byte sens)
    {
        /*
        short value: 409.5, short >= 46.5ma 3.3v
        20.3ma value: 253.8, 2.31v
        slight pressure: 53.9
        atmospheric pressure=4ma, 4.04ma value: 51.1-53.1, 0.58v
        slight vacuum: 43.5
        disconnect value: 0, v0 0ma
        */
        // sensors now start from 1 (but arrays from 0)
        sens--;
        myPS *p = &psensor[sens];
        // https://forum.arduino.cc/index.php?topic=571166.0
        analogRead(p->_sensorPin); // discard first value
        // convert analog value to pressure
        LOG.printf("measuring pSensor %u @pin%u calib:%u value:", sens + 1, p->_sensorPin, analogRead(p->_sensorPin)); // second value to serial mon
        uint16_t pres = (analogRead(p->_sensorPin) / p->_divider + p->_offset);                                    // third value goes into evaluation
        p->_last_pressure = pres; // storing value for reuse
        LOG.println(pres);

        //2  overpressure
        if (pres > p->_max_pressure && pres < 255)
        {
            // stop pump
            expanders.Pump(false);
            Alarm.delay(100);
            expanders.Protect(true);
            // set error
            p->_ErrorState = 2;
            LOG.printf("[E] sensor %u overpressure\r\n", sens + 1);
            SendSMS("Overpressure @pressure sensor", sens + 1);
            return pres; // exits here
        }

        //4  short circuit
        if (pres > 255)
        {
            // stop pump
            expanders.Pump(false);
            Alarm.delay(100);
            expanders.Protect(true);
            // set error
            p->_ErrorState = 4;
            LOG.printf("[E] sensor %u short circuit\r\n", sens + 1);
            //send sms
            SendSMS("short circuit @pressure sensor", sens + 1);
            return pres; // exits here
        }

        //3  disconnected
        if (pres < 20)
        {
            // stop pump
            expanders.Pump(false);
            Alarm.delay(100);
            expanders.Protect(true);
            // set error
            p->_ErrorState = 3;
            LOG.printf("[E] sensor %u disconnected\r\n", sens + 1);
            //send sms
            SendSMS("lost connection @pressure sensor", sens + 1);
            return pres; // exits here
        }

        //1  no pressure
        if (pres < p->_min_pressure)
            if (p->_ErrorState == 0)
                p->_ErrorState = 1;

        //0  normal pressure range is here - reset errors?
        if (pres > p->_min_pressure && pres < p->_max_pressure)
            if (p->_ErrorState == 1)
                p->_ErrorState = 0;

        // "add global pressure error here" - what is this??
        if (p->_ErrorState > 1)
        { // critical error
            if (sens == 0)
                SystemState.error_set(ERR_WATER_PRESSURE);
            else
                SystemState.error_set(ERR_NUTRI_PRESSURE);
        }

        return pres;
    }

    // error handling
    //0  normal pressure
    //1  no pressure
    //2  overpressure
    //3  disconnected
    //4  short circuit
    byte ErrorGet(byte sens) 
    { 
        // sensors now start from 1 (but arrays from 0)
        sens--;
        return psensor[sens]._ErrorState; 
    }

    // disable protect
    // unset system error
    // remeasure
    void ErrorReset(byte sens)
    {
        // sensors now start from 1 (but arrays from 0)
        sens--;
        expanders.Protect(false);
        psensor[sens]._ErrorState = 0;
        if (!sens) // sensor-0 freshwater
            SystemState.error_unset(ERR_WATER_PRESSURE);
        else // sensor-1 nutrients
            SystemState.error_unset(ERR_NUTRI_PRESSURE);
        // back to one - cause we calling function that also counts from one
        sens++;
        measure(sens);
    }

} pressure;

struct myBR
{
    //bit field
    //76543210
    //0 0x1  - flush solenoid error
    //1 0x2  - store solenoid error
    //2 0x4  - drain solenoid error
    //3 0x8  - Ultrasonic sensor checksum error
    //4 0x10 - Ultrasonic sensor  timeout error
    //5 0x20 -
    //6 0x40 - disabled manually
    //7 0x80 - other error
    byte _ErrorState = 0;
    float _Concentraion = 0;            // percent of nutrients in solution
    float _ConcentraionLast = 0;        // percent of nutrients in solution
    uint16_t _VolumeFreshwater = 0;     // data from flow sensor
    uint16_t _VolumeFreshwaterLast = 0; // data from flow sensor
    uint16_t _VolumeNutrients = 0;      // data from flow sensor
    uint16_t _VolumeNutrientsLast = 0;  // data from flow sensor
    uint16_t _SonicLastValue = 0;       // updated on every sonic measurement
    float _SonicLastError = 0;          // +- percent error in measurement
    byte _SonicHighErrors = 0;          // error > 5% last few times? how to calc?
                                        //static calibration values - set once at calibration
    // calibrate by filling and draining
    uint16_t _VolumeMin = 0; // for flow and sonic sensors
    // calibrate by filling untill miscalculation
    uint16_t _VolumeMax = 1; // for flow and sonic sensors
    // calibrate by filling 100L
    // also can be calculated by barrel diameter
    uint16_t _SonicMLinMM = 1130; // mililiters in 1 milimeter
    // calibrate by dry barrel height
    uint16_t _SonicOffset = 1000; // mm to barrel's full point
};

class BARRClass
{
private:
    myBR myBarrel[8];

public:
    bool LoadSD() { return Load("/Barrels.bin", (byte *)&myBarrel, sizeof(myBarrel)); }

    bool SaveSD() { return Save("/Barrels.bin", (byte *)&myBarrel, sizeof(myBarrel)); }

    // error handling
    byte ErrorGet(byte barrel) { return myBarrel[barrel]._ErrorState; }
    bool ErrorCheck(byte barrel, byte mask) { return myBarrel[barrel]._ErrorState & mask; }
    void ErrorSet(byte barrel, byte mask)
    {
        myBarrel[barrel]._ErrorState |= mask;
        LOG.printf("[E] Barr%u:e%u\r\n", barrel, mask);
    }
    void ErrorUnset(byte barrel, byte mask) { myBarrel[barrel]._ErrorState &= ~mask; }
    void ErrorReset(byte barrel) { myBarrel[barrel]._ErrorState = 0; }

    // freshwater counted by flow for (barrel)
    uint16_t FreshGet(byte barrel) { return myBarrel[barrel]._VolumeFreshwater; }

    // nutrients counted by flow for (barrel)
    uint16_t NutriGet(byte barrel) { return myBarrel[barrel]._VolumeNutrients; }

    // add fresh flow couter to barrel, then Subtract it from flowsensor
    void FreshwaterFillCalc(byte barrel)
    {
        myBR *b = &myBarrel[barrel];
        if (b->_VolumeFreshwaterLast != b->_VolumeFreshwater) // safeguard
            b->_VolumeFreshwater = b->_VolumeFreshwaterLast;
        else
            b->_VolumeFreshwaterLast = b->_VolumeFreshwater;

        uint32_t tempflow = flow.CounterGet(FLOW_SENSOR_FRESHWATER); // may be increased during calculation because flowsensor works on interrupts
        if (tempflow)
        {
            LOG.printf("tempflow so far: %u\r\n", tempflow);
            tempflow /= flow.DividerGet(FLOW_SENSOR_FRESHWATER); // integral part, fractional part discarded.
            LOG.printf("tempflow in liters: %u\r\n", tempflow);
            b->_VolumeFreshwater += tempflow;
            tempflow *= flow.DividerGet(FLOW_SENSOR_FRESHWATER); // getting pulse count back - only the integral part
            LOG.printf("tempflow integral part: %u\r\n", tempflow);
            flow.CounterSubtract(FLOW_SENSOR_FRESHWATER, tempflow);              // counter 1 is freshwater
        }
        else
        {
            LOG.println("0 flow so far. skipping");
        }
        b->_VolumeFreshwaterLast = b->_VolumeFreshwater; // after
    }

    //https://sciencing.com/calculate-concentration-solution-different-concentrations-8680786.html
    // concentration of nutrient solution
    // ((concA/100 *LitersA) + (concB/100 *LitersB)) / LitersA + LitersB  * 100%
    // concB is allways 0 ( b is freshwater) since that is irrelevant, so:
    // (concA/100 *LitersA)  / LitersA + LitersB  * 100%
    // concA * LitersA  / LitersA + LitersB
    float ConcentrationTotal(byte barrel)
    {
        myBR *b = &myBarrel[barrel];
        if (!b->_VolumeNutrients && !b->_VolumeFreshwater)
            return 0; // empty barrels - prevent Divide By Zero!
        else
            return b->_Concentraion * b->_VolumeNutrients / (b->_VolumeNutrients + b->_VolumeFreshwater); // mix
    }

    // calc new concentration:
    // counts all as nutrinets after calculation, at concentration X
    void ConcentrationRecalc(byte barrel)
    {
        myBR *b = &myBarrel[barrel];
        if (b->_VolumeFreshwater)
        {
            if (b->_ConcentraionLast != b->_Concentraion)
                b->_Concentraion = b->_ConcentraionLast; // safeguard
            b->_Concentraion = ConcentrationTotal(barrel);
            b->_VolumeNutrients += b->_VolumeFreshwater;
            b->_VolumeFreshwater = 0;
            b->_ConcentraionLast = b->_Concentraion;
        }
        else
        {
            LOG.println(F("[W] ConcentrationRecalc called but already calculated"));
        }
    }

    // checks if current barrel freshwater/nutrients level is at/above target
    // uses barrel current level + flow counted level
    // does not apply or decrease flow count!
    bool isFillTargetReached(byte barrel, byte type, uint16_t target)
    {
        myBR *b = &myBarrel[barrel];
        uint16_t tempLiters = 0;
        //check target barrel current state
        if (type == BARREL_FRESHWATER)
            tempLiters += b->_VolumeFreshwater;
        else
            tempLiters += b->_VolumeNutrients;
        //add data from flowsensor
        tempLiters += flow.CounterGet(type) / flow.DividerGet(type);
        //check if target reached
        return tempLiters >= target;
    }

    // Subtracting flowcount from source, adding to destination, recalculating concentration @destination
    // run less often for concentration accuracy
    void NutrientsTransferCalc(byte from, byte to)
    {
        myBR *a = &myBarrel[from];
        myBR *b = &myBarrel[to];
        // calc new concentration before transer so transfeting only nutrients @concentration
        if (a->_VolumeFreshwater)
            ConcentrationRecalc(from);
        if (b->_VolumeFreshwater)
            ConcentrationRecalc(to);
        // both barrels now have only Nutrients @ concentration

        // safeguards below
        if (a->_VolumeNutrientsLast != a->_VolumeNutrients)
            a->_VolumeNutrients = a->_VolumeNutrientsLast;
        else
            a->_VolumeNutrientsLast = a->_VolumeNutrients;

        if (b->_VolumeNutrientsLast != b->_VolumeNutrients)
            b->_VolumeNutrients = b->_VolumeNutrientsLast;
        else
            b->_VolumeNutrientsLast = b->_VolumeNutrients;

        if (a->_ConcentraionLast != a->_Concentraion)
            a->_Concentraion = a->_ConcentraionLast;
        else
            a->_ConcentraionLast = a->_Concentraion;

        if (b->_ConcentraionLast != b->_Concentraion)
            b->_Concentraion = b->_ConcentraionLast;
        else
            b->_ConcentraionLast = b->_Concentraion;

        uint32_t tempflow = flow.CounterGet(FLOW_SENSOR_NUTRIENTS); // may be increased during calculation because flowsensor works on interrupts
        if (tempflow)
        {
            tempflow /= flow.DividerGet(FLOW_SENSOR_NUTRIENTS);         // integral part, fractional part discarded.
            LOG.printf("transfering %u liters from barrel %u to %u\r\n", tempflow, from, to);
            LOG.printf("source barrel %u before: %u, target barrel %u before: %u\r\n", from, a->_VolumeNutrients, to, b->_VolumeNutrients);
            // barrel b concentration =  (concentrationA/100 *Aliters) + (concentrationB/100 *Bliters)   / (Aliters + Bliters)  * 100%
            // recheck this implementation !!! should I calc before adding counter?
            LOG.printf("concentration before: %f\r\n", b->_Concentraion);
            b->_Concentraion = ((a->_Concentraion / 100 * tempflow) + (b->_Concentraion / 100 * b->_VolumeNutrients)) / (tempflow + b->_VolumeNutrients) * 100;
            // recheck this implementation !!!
            // is "_Concen /100 * tflow" same as "_Concen * tflow /100" considering uneven float point calculation?
            // "Do not use float to represent whole numbers." http://www.cplusplus.com/forum/general/67783/

            a->_VolumeNutrients -= tempflow;
            b->_VolumeNutrients += tempflow;
            tempflow *= flow.DividerGet(FLOW_SENSOR_NUTRIENTS);     // getting pulse count back - only the integral part
            flow.CounterSubtract(FLOW_SENSOR_NUTRIENTS, tempflow); // counter 2 is nutrients
        }
        else
        {
            LOG.println("0 flow so far. skipping");
        }

        a->_VolumeNutrientsLast = a->_VolumeNutrients;
        b->_VolumeNutrientsLast = b->_VolumeNutrients;
        a->_ConcentraionLast = a->_Concentraion;
        b->_ConcentraionLast = b->_Concentraion;
        if (tempflow)
        {
            LOG.printf("source barrel %u after: %u, target barrel %u after: %u\r\n", from, a->_VolumeNutrients, to, b->_VolumeNutrients);
            LOG.printf("concentration after: %f\r\n", b->_Concentraion);
        }
    }

    // decrements barrel by drained liters, Flowcounter2, global drain_requirement (except in manual mode)
    // increments grand total global drain_counter
    void DrainLitrageSubtract(byte barrel, uint32_t liters)
    {
        // if not manual mode - decrement global drain requirement
        if (!SystemState.state_check(MANUAL_STATE)) 
            if (Transfers.drain_requirement) // prevent integer overflow
                Transfers.drain_requirement -= liters; //for need-to-drain check in the fmsTask function?
        // increment grand total drain
        if (Transfers.drain_counter<UINT32_MAX) // prevent integer overflow
            Transfers.drain_counter += liters;
        // decrement the barrel you draining from
        myBarrel[barrel]._VolumeNutrients -= liters;
        
        liters *= flow.DividerGet(FLOW_SENSOR_NUTRIENTS);     // getting pulse count back - only the integral part
        flow.CounterSubtract(FLOW_SENSOR_NUTRIENTS, liters); // counter 2 is nutrients
    }



    // save at each change? inside the changing function? only to SD?
    // what minimal SD write size? 512 bytes?

    // 0 to 100% from flow

    uint16_t VolumeMax(byte barrel)
    {
        return myBarrel[barrel]._VolumeMax;
    }

    uint16_t VolumeMin(byte barrel)
    {
        return myBarrel[barrel]._VolumeMin;
    }

    // contact the sensor via UART, measure, return distance in mm
    void SonicMeasure(byte barrel, byte measure = 10, uint16_t timeLeft = 1000, byte retryLeft = 5)
    { // the hedgehog :P
        uint16_t temptimeout = timeLeft;
        byte tempretry = retryLeft;
        myBR *b = &myBarrel[barrel];
        // collect "measure" successful measurements to calculate total
        // wait "timeLeft" ms total time for sonic data
        // try "retryLeft" times on no data or checksum error
        uint32_t distanceAvearge = 0;  // avearge of x measurements
        uint16_t distanceMin = 0xffff; // highest for 16bit uint
        uint16_t distanceMax = 0;      // to calculate error
        LOG.printf("measuring barrel# %u\r\n", barrel);
        expanders.setMUX(barrel);
        delay(1);
        for (byte x = 0; x < measure && retryLeft && timeLeft;)
        {
            // send data so sonic will reply
            Serial2.write(0xFF);
            // wait untill some data is received
            while (!Serial2.available() && timeLeft)
            {
                delay(1);
                timeLeft--;
            };
            // discard data untill begin of packet (0xFF)
            while (Serial2.read() != 0xFF && timeLeft)
            {
                delay(1);
                timeLeft--;
            };
            // wait for all data to be buffered
            while (Serial2.available() < 3 && timeLeft)
            {
                delay(1);
                timeLeft--;
            };
            // timed out waiting for 3 packats above
            if (!timeLeft)
            {
                // number of measurements so far (excluding the last "timed out" measurement)
                measure = x;
                ErrorSet(barrel, BARREL_SONIC_TIMEOUT);
                break;
            }
            byte upper_data = Serial2.read();
            byte lower_data = Serial2.read();
            byte sum = Serial2.read();
            //Serial.printf("high %u low %u sum %u\r\n", upper_data, lower_data, sum);
            if (((upper_data + lower_data) & 0xFF) == sum)
            {
                uint16_t distance = (upper_data << 8) | (lower_data & 0xFF);
                LOG.printf("Sonic:%u Distance:%umm measurement:%u time left:%u retries left:%u\r\n", barrel, distance, x, timeLeft, retryLeft);
                if (distance)
                {
                    if (distance == 10555)
                    {
                        measure = x; // number of measurements so far (excluding the last "out of range" measurement)
                        distanceAvearge = 0;
                        ErrorSet(barrel, BARREL_SONIC_OUTOFRANGE);
                        break;
                    }
                    else
                    {
                        distanceAvearge += distance;
                        if (distanceMin > distance)
                            distanceMin = distance;
                        if (distanceMax < distance)
                            distanceMax = distance;
                        x++; // success - decrement loop counter
                    }
                }
                else
                    retryLeft--;
            }
            else
            {
                LOG.println("checksum error");
                retryLeft--;
            }
            if (!retryLeft)
                ErrorSet(barrel, BARREL_SONIC_CHECKSUM);
            else
                ErrorUnset(barrel, BARREL_SONIC_CHECKSUM);
        }                      // for loop end here
        expanders.UnlockMUX(); // important!
        float err = 0;
        if (measure && distanceAvearge)
        {                                                                           // taken more than 0 measurements, Avearge distance is not zero
            distanceAvearge /= measure;                                             // total divided by number of measurements taken
            err = (float)100 * ((distanceMax - distanceMin) / 2) / distanceAvearge; // calculate measurement ±error
            ErrorUnset(barrel, BARREL_SONIC_TIMEOUT);
            ErrorUnset(barrel, BARREL_SONIC_OUTOFRANGE);
            b->_SonicLastValue = distanceAvearge; // set value to be used by other functions. will leave previous if measurement was bad.
            LOG.printf("Distance min:%u, max:%u, diff:%u error:±%.3f percent\r\n",
                          distanceMin,
                          distanceMax,
                          distanceMax - distanceMin,
                          err);
        }
        b->_SonicLastError = err;
        if (err > 5)
        { // if error > ±5%
            if (b->_SonicHighErrors < 30)
                b->_SonicHighErrors += 10;
            // increment by 10 each measurement error, decrement by 1 each success
            else
                ErrorSet(barrel, BARREL_SONIC_INACCURATE);
        }
        else if (b->_SonicHighErrors) // if not err>5, and if errors not already zero, decrement
            b->_SonicHighErrors--;
        else
            ErrorUnset(barrel, BARREL_SONIC_INACCURATE);
        LOG.printf("sonic %u finished\r\ntook %u measurements, value:%umm time:%ums, retried %u times\r\n",
                      barrel,
                      measure,
                      distanceAvearge,
                      temptimeout - timeLeft,
                      tempretry - retryLeft);
        //return distanceAvearge;
    } // end SonicMeasure

    uint16_t SonicLastMM(byte barrel) { return myBarrel[barrel]._SonicLastValue; }
    int16_t SonicLastMMfromEmpty(byte barrel) { return myBarrel[barrel]._SonicOffset - myBarrel[barrel]._SonicLastValue; }
    uint16_t SonicOffsetGet(byte barrel) { return myBarrel[barrel]._SonicOffset; }
    uint16_t SonicMLinMMGet(byte barrel) { return myBarrel[barrel]._SonicMLinMM; }
    void SonicOffsetSet(byte barrel, uint16_t offs) { myBarrel[barrel]._SonicOffset = offs; }
    void SonicMLinMMSet(byte barrel, uint16_t coef) { myBarrel[barrel]._SonicMLinMM = coef; }

    // sonic calculate liters of last measurement
    int16_t SonicCalcLiters(byte barrel)
    {
        // _SonicOffset = empty barrel (full barrel lenght) in mm
        // "full barrel lenght" - SonicMeasure = water level from empty in mm
        // lenght * _SonicMLinMM / 1000mlINliter ) = current barrel volume in liters from sonic
        myBR *b = &myBarrel[barrel];
        //Serial.printf("barrel %u (offset %u - value %u) * MLinMM %u / 1000ml\r\n", barrel, b->_SonicOffset, b->_SonicLastValue, b->_SonicMLinMM);
        if (!b->_SonicLastValue)
            return 0;
        else
            return (b->_SonicOffset - b->_SonicLastValue) * b->_SonicMLinMM / 1000; // 1000ml in liter
    }                                                                               // should I still return positive value for empty, but not dry barrel??

    // barrel current volume in percent by sonic the hedgehog
    // no need to remeasure each call - needed real-time only at transfer - so updated anyway by transfer volume checkers below
    // 100% * "current liters above min point" / "total liters above min point"
    int8_t BarrelPercents(byte barrel)
    {
        return 100 * (SonicCalcLiters(barrel) - myBarrel[barrel]._VolumeMin) / (myBarrel[barrel]._VolumeMax - myBarrel[barrel]._VolumeMin);
        // exclude unusable percents below Min point
    }

    // total liters in all barrels excluding mixing barrel and barrels with errors
    // should I remeasure all sonics before?
    int16_t SonicLitersTotal()
    {
        int16_t result = 0;
        for (byte x = 1; x < NUM_OF_BARRELS; x++)
            if (!ErrorGet(x))                 // if no errors at all
                result += SonicCalcLiters(x); // add this barrel content to sum
        return result;
    }

    // same as above but exclude unusable liters from below draining point.
    // should I remeasure all sonics before?
    int16_t SonicLitersUsable()
    {
        int16_t result = 0;
        for (byte x = 1; x < NUM_OF_BARRELS; x++)
            if (!ErrorGet(x))                                            // if no errors at all
                result += (SonicCalcLiters(x) - myBarrel[x]._VolumeMin); // add this barrel content minux wasted liters to sum
        return result;
    }

    // totally empty (by ultrasonic)
    bool isDry(byte barrel)
    {
        SonicMeasure(barrel);
        return SonicCalcLiters(barrel) < 2; // +1 spare as a safeguard
    }

    // reached min level (by ultrasonic)
    bool isEmpty(byte barrel)
    {
        SonicMeasure(barrel);
        return SonicCalcLiters(barrel) <= myBarrel[barrel]._VolumeMin;
    }

    // reached max level (by ultrasonic)
    bool isFull(byte barrel)
    {
        SonicMeasure(barrel);
        return SonicCalcLiters(barrel) >= myBarrel[barrel]._VolumeMax;
    }

} barrels;



// receives barrel to fill, required water level
// checks whatever clean water line have pressure
// fills clean water until level is reached, or barrel is full, or until flowsensor malfunction detected
void Fill(byte barrel, uint16_t requirement)
{
    LOG.printf("filling barrel:%u to %uL\r\n", barrel, requirement);
    // reset flow counter 1
    flow.CounterReset(FLOW_SENSOR_FRESHWATER);
    byte waited_for_flow = 0; // number of seconds without flow
    pressure.measure(PRES_SENSOR_FRESHWATER);      //freshwater at sensor 1
    if (pressure.ErrorGet(PRES_SENSOR_FRESHWATER) == PRESSURE_NOPRESSURE)
    {
        // water line no pressure? stop, set error,
        SystemState.error_set(ERR_WATER_PRESSURE);
        // sendsms
        SendSMS("no water pressure. system paused.");
        // recheck in loop - if ok - clear error
        while (pressure.ErrorGet(PRES_SENSOR_FRESHWATER) != PRESSURE_NORMAL)
        {
            Alarm.delay(1000);
            pressure.measure(PRES_SENSOR_FRESHWATER);
            Alarm.delay(1000);
        }
        SystemState.error_unset(ERR_WATER_PRESSURE);
    }

    // open barrel filling tap
    expanders.FillingRelay(barrel, true);

    // fill until requirement OR barrel_high_level
    while (!barrels.isFillTargetReached(barrel, BARREL_FRESHWATER, requirement))
    { 

        // LOGIC
        // assign flowcount to barrel
        barrels.FreshwaterFillCalc(barrel);         
        // if barrel ammount is Full then stop
        if (barrels.isFull(barrel))
        {
            LOG.printf("[E] barrel:%u full. breaking..\r\n", barrel);
            break;
        }

        // STOP-BREAK
        // check if changed to STOPPED state
        if (SystemState.state_check(STOPPED_STATE) && !SystemState.state_check(MANUAL_STATE)) // break if stopped but not manual
        {
            LOG.println("Status Stopped and not Manual. breaking.");
            break;                                                                            // breaks if stopped set
        }


        // SENSOR CHECK
        // check flow - if no flow (but pressure) for 10 times (10 seconds)
        pressure.measure(PRES_SENSOR_FRESHWATER); // will stop at overpressure
        if (!flow.FlowGet(FLOW_SENSOR_FRESHWATER)) // noflow
            if (pressure.ErrorGet(PRES_SENSOR_FRESHWATER) == PRESSURE_NORMAL) // normal pressure
            {
                if (waited_for_flow < 255)
                    waited_for_flow++; // will increase by 1 every second where is no flow
                else
                    waited_for_flow = 0; // reset count if flow detected
            }
        // if no flow set flowsensor1 error
        if (waited_for_flow >= 10)
        { // 10 seconds or more
            SystemState.error_set(ERR_WATER_NOFLOW);
            SendSMS("error: no water flow while [Filling]. check flowsensor 1, filling solenoid");
            SystemState.state_set(STOPPED_STATE); // untill separate error-handling reimplemented
            break;                                // breaks + sets stopped if flowsensor error
        }

        Alarm.delay(1000);
    }
    // close tap
    expanders.FillingRelay(barrel, false);
    // assign flowcount to barrel
    barrels.FreshwaterFillCalc(barrel); 
    // reset flow counter 1
    flow.CounterReset(FLOW_SENSOR_FRESHWATER);
    LOG.printf("END filling barrel:%u to %uL\r\n", barrel, requirement);
}

// will mix "barrel" untill "duration" minutes is over,
// or before if state changed to stopped while we're not operating manual
void Mix(byte barrel, uint16_t duration)
{
    LOG.printf("mixing barrel:%u for %uMin.\r\n", barrel, duration);
    // reset flow counter 2
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS);
    // open barrel drain tap
    expanders.DrainingRelay(barrel, true);
    // open barrel store tap
    expanders.StoringRelay(barrel, true);
    // start pump
    expanders.Pump(true);
    // seconds
    byte sec = 0;    
    // loops with no pressure
    byte nopres = 0;
    // loop - while counter > 0
    while (duration)
    {

        // LOGIC
        // decrement counter every minute
        if (sec < 60)
        {
            Alarm.delay(1000);
            sec++;
        }
        else
        {
            duration--;
            sec = 0;
            LOG.printf("barrel:%u %uMin remaining\r\n", barrel, duration);
        }


        // STOP-BREAK
        // break if stopped but not manual
        if (SystemState.state_check(STOPPED_STATE) && !SystemState.state_check(MANUAL_STATE))
        {
            LOG.println("Status Stopped and not Manual. breaking.");
            break; // breaking the measure loop will close taps
        }


        // SENSOR CHECK
        // check pressure in range
        pressure.measure(PRES_SENSOR_NUTRIENTS); 
        if (pressure.ErrorGet(PRES_SENSOR_NUTRIENTS) == PRESSURE_NOPRESSURE)
            nopres++; // increment every cycle (second) of no pressure
        else
            nopres = 0; // reset if measured ok once
        if (nopres > 4)
        {
            SystemState.error_set(ERR_NUTRI_PRESSURE);
            SendSMS("error: no pressure while [Mixing]. check pump, solenoids, and sensors. barrel:", barrel);
            SystemState.state_set(STOPPED_STATE); // untill separeate error-handling reimplemented
            break;                                // breaks + sets stopped if sensor error
        }


    }
    // counter reached zero
    // stop pump
    expanders.Pump(false);
    // wait untill pressure released
    Alarm.delay(100);
    // close barrel drain tap
    expanders.DrainingRelay(barrel, false);
    // close barrel store tap
    expanders.StoringRelay(barrel, false);
    // reset flow counter 2
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS);
    LOG.printf("END mixing barrel:%u for %uMin.\r\n", barrel, duration);
}

// transfers nutrients from "barrel" to "target"
// untill barrel is empty or untill target is full
// or untill "stopped" state set, except if system state also set to manual
void Store(byte barrel, byte target)
{
    LOG.printf("storing from barrel:%u to target %u\r\n", barrel, target);
    // reset flow counter 2
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS);
    if (barrel == target)
    {
        LOG.println("Error! trying to store to itself.\r\nstoring function exit now.");
        return;
    }
    // open barrel drain tap
    expanders.DrainingRelay(barrel, true);
    // open target store tap
    expanders.StoringRelay(target, true);
    // start pump
    expanders.Pump(true);
    byte nopres = 0; // loops with no pressure
    // loop - while source barrel not empty AND target not full
    while (!barrels.isEmpty(barrel) && !barrels.isFull(target))
    {

        // LOGIC
        // every 50 liters (not too often for good calculation accuracy)
        if (flow.CounterGet(FLOW_SENSOR_NUTRIENTS) / flow.DividerGet(FLOW_SENSOR_NUTRIENTS) > 50)
        {
            // truncate flow counter from barrel
            // append flow counter to target
            // also prints useful info to serial monitor
            barrels.NutrientsTransferCalc(barrel, target);
        }

        // STOP-BREAK
        // break if stopped but not manual
        if (SystemState.state_check(STOPPED_STATE) && !SystemState.state_check(MANUAL_STATE))
        {
            LOG.println("Status Stopped and not Manual. breaking.");
            break; // breaking the measure loop will close taps
        }

        // SENSOR CHECK
        //evaluate pressure in loop
        pressure.measure(PRES_SENSOR_NUTRIENTS); //nutrients at sensor 2
        if (pressure.ErrorGet(PRES_SENSOR_NUTRIENTS) == PRESSURE_NOPRESSURE)
            nopres++; // increment every cycle (second) of no pressure
        else
            nopres = 0; // reset if measured ok once
        if (nopres > 40) // 4 seconds
        {
            SystemState.error_set(ERR_NUTRI_PRESSURE);
            SendSMS("error: no pressure while [Storing]. check pump, solenoids, and sensors");
            SystemState.state_set(STOPPED_STATE); // untill separeate error-handling reimplemented
            break;                                // breaks + sets stopped if flowsensor error
        }
        // in next version:
        //if overpressure error - set target barrel error
        //goto next barrel, clear overpressure error
        //if all barrels error - set pump error - clear all barrel overpressure errors
        //  except manually-disabled barrels "error"
        //if no pressure - loop-measure for a while
        //if still no pressure - check flow
        //no flow - set pump error.

        Alarm.delay(100);
    }
    // stop pump
    expanders.Pump(false);
    // wait untill pressure released
    Alarm.delay(100);
    // close barrel drain tap
    expanders.DrainingRelay(barrel, false);
    // close target store tap
    expanders.StoringRelay(target, false);
    // calculate final ammount
    barrels.NutrientsTransferCalc(barrel, target);
    // reset flow counter 2
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS);
    LOG.printf("END storing from barrel:%u to target %u\r\n", barrel, target);
}

// draining from barrel, untill requirement liters is transferred, or barrel is empty
// or untill state set to stopped except if state is also manual
void Drain(byte barrel, uint16_t requirement)
{
    LOG.printf("Draining %uL from barrel:%u\r\n", requirement, barrel);
    // reset flow counter 2
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS);
    // open barrel drain tap
    expanders.DrainingRelay(barrel, true);
    // open pools "store" tap
    expanders.StoringRelay(6, true); // store No6 is pools out
    // start pump
    expanders.Pump(true);
    // loops with no pressure
    byte nopres = 0; 
    // liters temp
    uint32_t tempflow = 0;
    // loop while drain counter > 0 and barrel x not empty
    while (requirement && !barrels.isEmpty(barrel))
    {
        // LOGIC
        tempflow = flow.CounterGet(FLOW_SENSOR_NUTRIENTS); // may be increased during calculation because flowsensor works on interrupts
        tempflow /= flow.DividerGet(FLOW_SENSOR_NUTRIENTS);         // integral part in liters, fractional part discarded.
        // flow counted a liter or more
        if (tempflow)
        {
            // truncate flow counter from a barrel + decrement flowcounter
            barrels.DrainLitrageSubtract(barrel, tempflow);
            // decrement requirement by flow counter
            requirement -= tempflow;
        }


        // STOP-BREAK
        // check for STOPPED state change
        if (SystemState.state_check(STOPPED_STATE) && !SystemState.state_check(MANUAL_STATE)) // break if stopped but not manual
        {
            LOG.println("Status Stopped and not Manual. breaking.");
            break;                                                                            // breaks the measure loop, stops pump, not decrement the x
        }                                                                                     //stays in the while STOPPED above

        // SENSOR CHECK
        // check pressure in range
        pressure.measure(PRES_SENSOR_NUTRIENTS); 
        if (pressure.ErrorGet(PRES_SENSOR_NUTRIENTS) == PRESSURE_NOPRESSURE)
            nopres++; // increment every cycle (second) of no pressure
        else
            nopres = 0; // reset if measured ok once
        if (nopres > 40) // 4 seconds (40 delays of 100ms below)
        {
            SystemState.error_set(ERR_NUTRI_PRESSURE);
            SendSMS("error: no pressure while [Draining]. check pump, solenoid, and sensors");
            SystemState.state_set(STOPPED_STATE); // untill separeate error-handling reimplemented
            break;                                // breaks + sets stopped if flowsensor error
        }

        Alarm.delay(100);
    }
    // stop pump
    expanders.Pump(false);
    // close barrel x tap
    expanders.DrainingRelay(barrel, false);
    // wait untill pressure released
    Alarm.delay(100);
    // close dOUT tap
    expanders.StoringRelay(6, false);
    Alarm.delay(1000);
    // last flow calculation
    tempflow = flow.CounterGet(FLOW_SENSOR_NUTRIENTS); // may be increased during calculation because flowsensor works on interrupts
    tempflow /= flow.DividerGet(FLOW_SENSOR_NUTRIENTS);         // integral part in liters, fractional part discarded.
    if (tempflow) // if more than liter - Subtract
        barrels.DrainLitrageSubtract(barrel, tempflow);
    // reset flow counter 2
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS);
    LOG.printf("END Draining %uL from barrel:%u\r\n", requirement, barrel);
}

void fmsTask()
{ // Filling Mixing Storing Draining
    LOG.printf("system state:%u\r\n", SystemState.state_get());
    while (true)
    {
        while (SystemState.state_check(STOPPED_STATE)) // stay here if system stopped
            Alarm.delay(1000);                         // check every second

        LOG.printf("system state:%u\r\n", SystemState.state_get());
        if (SystemState.state_check(FILLING_STATE))
        {
            //Fill(Transfers.filling_barrel, Transfers.prefill_requirement);
            SystemState.state_set(STOPPED_STATE); // wait untill nutes loaded before filling+mixing
            Fill(Transfers.filling_barrel, Transfers.afterfill_requirement);
            SaveStructs();
            Mix(Transfers.filling_barrel, Transfers.mix_requirement);
            SystemState.state_set(STORING_STATE);
            SystemState.state_unset(FILLING_STATE);
            SaveStructs();
        }

        LOG.printf("system state:%u\r\n", SystemState.state_get());
        if (SystemState.state_check(STORING_STATE))
        {
            // still have nutes to transfer
            while (!barrels.isEmpty(Transfers.filling_barrel))
            {
                // first we drain if neccesery
                if (Transfers.drain_requirement)
                {
                    // drain untill empty or requirement satisfied.
                    Drain(Transfers.filling_barrel, Transfers.drain_requirement);
                    if (barrels.isEmpty(Transfers.filling_barrel))
                        break; // if drained filling barrel to empty - break store loop
                }
                // then we store what is left
                // target not full - transfer into storing barrel
                // excluding the case where all system was full and storing_barrel pointed to barrel 0
                // skip barrel if disabled or errorous!
                if (!barrels.isFull(Transfers.storing_barrel) && !barrels.ErrorGet(Transfers.storing_barrel) && Transfers.storing_barrel > 0)
                    Store(Transfers.filling_barrel, Transfers.storing_barrel);
                // target full - goto next barrel
                else if (Transfers.storing_barrel > 1)
                {
                    Transfers.storing_barrel--;
                }
                else
                { // no more next - all full
                    expanders.setRGBLED(LED_CYAN);
                    LOG.println("All barrels full. system stopped.");
                    // wait for drain request
                    while (!Transfers.drain_requirement)
                        Alarm.delay(1000);
                    // drain the mixer first
                    Transfers.storing_barrel = Transfers.filling_barrel;
                    // drain untill empty or requirement satisfied.
                    Drain(Transfers.filling_barrel, Transfers.drain_requirement);
                }
            } // got here cause filling_barrel is empty
            // or broke out of the loop cause all including mixer is full
            while (Transfers.drain_requirement)
            {
                // storing_barrel not empty? not errorous? drain it
                if (!barrels.isEmpty(Transfers.storing_barrel) && !barrels.ErrorGet(Transfers.storing_barrel))
                    Drain(Transfers.storing_barrel, Transfers.drain_requirement);
                // storing_barrel empty but not the last barrel (i filled from last to first)  // try next barrel
                else if (Transfers.storing_barrel < NUM_OF_BARRELS - 1)
                    Transfers.storing_barrel++;
                // all storage barrels empty?
                else
                    break; // totally empty - will do another cycle fms to refill
            }
            // repeat the fms cycle if no draining required, or all barrels empty
            SystemState.state_set(FILLING_STATE);
            SystemState.state_unset(STORING_STATE);

            SaveStructs();
        } // if storing_state
    }     // endless loop ends here

    //drain should be launched on-demand only?

    // filling and mixing barrel always the same (barrel 0)
    // storing and draining barrel always the same
    // storing to last barrel, going down in barrels while we full
    // if all full - storing barrel is mixing barrel
    // going up barrels while we drain
    // draining - last barrel empty - break - going back to fms,
    // doing another cycle, draining again in the end of the cycle

    // what should happen if trying to drain more than available?
    // right now drain is looping endlessly
    // can I pause drain and continue after one fms cycle?
}

// WebServer

//https://github.com/me-no-dev/ESPAsyncWebServer

void setupServer()
{

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        request->redirect("/manual"); //untill implemented
        /*
        if(!SD.exists("/index.html")){
            SD.end();
            LOG.println("trying to restart SD");
            if(!SD.begin(22)){
            LOG.println("unable to read form SD card");
            request->send(200, "text/html", "<html><body><center><h1>check SD Card please</h1></center></body></html>");
            }
            }
        request->send(SD, "/index2.html", String(), false);
        */
    });

    server.on(
        "/upload", HTTP_POST, [](AsyncWebServerRequest *request) {}, [](AsyncWebServerRequest *request, String filename, size_t index, byte *data, size_t len, bool final) {
            if(!index)
            {
                LOG.printf("\r\nUpload Started: %s\r\n", filename.c_str());
                // open the file on first call and store the file handle in the request object
                request->_tempFile = disk->open("/"+filename, "w");
            }
            if(len) 
            {
                LOG.printf("received chunk [from %u to %u]\r\n", index, len);
                // stream the incoming chunk to the opened file
                request->_tempFile.write(data,len);
            }       
            if(final)
            {
                LOG.printf("\r\nUpload Ended: %s, %u Bytes\r\n", filename.c_str(), index+len);
                request->_tempFile.close();
                request->redirect("/list");
                //request->send(200, "text/plain", "File Uploaded !");
            } });

    server.on("/list", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        AsyncResponseStream *response = request->beginResponseStream("text/html");
        File dir = disk->open("/");
        File file = dir.openNextFile();
        response->print("<html><body style=\"transform: scale(2);transform-origin: 0 0;\"><h3>file system</h3><ul>");
        while (file)
        {
            response->print("<li>");
            response->printf("<button onclick=\"location=\'/del?f=%s\'\">Delete</button><span>\t</span>", file.name());
            response->printf("<a href=\"down?f=%s\"><b>%s</b></a> \t%u bytes \t %li timestamp</li>", file.name(), file.name(), file.size(), file.getLastWrite());
            file.close();
            file = dir.openNextFile();
        }
        dir.close();
        response->print("</ul><form method='POST' action='/upload' enctype='multipart/form-data'>");
        response->print("<input type='file' name='update'><input type='submit' value='Upload'></form>");
        response->print("<button onclick=\"location=\'/backup\'\">Backup all to SPIFFS</button><span> </span>");
        response->print("<button onclick=\"location=\'/restore\'\">Restore missing to SD</button><br><br>");
        response->print("<button onclick=\"location=location\">reload</button><span> </span>");
        response->print("<button onclick=\"location=\'/reset\'\">reset</button><br><br>");
        response->print("<button onclick=\"location=\'/manual\'\">open manual control</button><br><br>");
        if (isSD)
        {
            response->print("<div>filesystem is: SD Card</div>");
            response->print("<button onclick=\"location=\'/switchFS\'\">Switch to SPIFFS</button><br><br>");
        }
        else
        {
            response->print("<div>filesystem is: SPIFFS</div>");
            response->print("<button onclick=\"location=\'/switchFS\'\">Switch to SD Card</button><br><br>");
        }
        response->print("</body></html>");
        request->send(response);
    });

    server.on("/switchFS", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (isSD)
        {
            LOG.println(F("switching to SPIFFS"));
            disk = &SPIFFS;
            isSD = false;
        }
        else
        { // IMPORTANT !! add check for filesystem availability
            LOG.println(F("switching to SD Card"));
            disk = &SD;
            isSD = true;
        }
        request->redirect("/list");
    });

    server.on("/del", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        if (request->args() > 0)
        { // Arguments were received
            if (request->hasArg("f"))
            {
                const char *file = request->arg("f").c_str();
                LOG.printf("Deleting file %s ", file);
                disk->remove(file) ? Serial.println("Successfully") : Serial.println("Failed");
            }
        }
        else
        {
            LOG.println("*server: del received no args");
        }
        request->redirect("/list");
    });

    server.on("/down", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        if (request->args() > 0)
        { // Arguments were received
            if (request->hasArg("f"))
            {
                const char *file = request->arg("f").c_str();
                LOG.printf("Downloading file %s \r\n", file);
                request->send(*disk, file, "text/plain");
            }
        }
        else
            request->send(200, "text/plain", "file not found");
    });

    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(404);
        //request->send(*disk, "/favicon.png", "image/png");
    });

    server.on("/manual", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n\r\n", request->url().c_str());
        AsyncResponseStream *response = request->beginResponseStream("text/html");
        response->print("<html><body style=\"transform: scale(2);transform-origin: 0 0;\"><h3>manual control</h3><ul>");
        response->print("<span>Relays</span>");
        for (byte i = 0; i < 8; i++)
        {
            response->print("<li>");
            response->printf("<button onclick=\"location=\'/man?f=%u&o=%u\'\">fill  %u %s </button><span> </span>", i, expanders.FillingRelayGet(i) ? 0 : 1, i, expanders.FillingRelayGet(i) ? "-ON" : "OFF");
            response->printf("<button onclick=\"location=\'/man?s=%u&o=%u\'\">store %u %s </button><span> </span>", i, expanders.StoringRelayGet(i) ? 0 : 1, i, expanders.StoringRelayGet(i) ? "-ON" : "OFF");
            response->printf("<button onclick=\"location=\'/man?d=%u&o=%u\'\">drain %u %s </button><span> </span>", i, expanders.DrainingRelayGet(i) ? 0 : 1, i, expanders.DrainingRelayGet(i) ? "-ON" : "OFF");
            response->print("</li>");
        }
        response->print("<span>RGB LED</span>");
        response->print("<li>");
        for (byte i = 0; i < 8; i++)
            response->printf("<button onclick=\"location=\'/man?rgb=%u\'\">RGB %u</button><span> </span>", i, i);
        response->print("</li>");
        response->print("<span>Flow Sensors</span>");
        response->printf("<li>Fs1: [%up] [%uL] [%umL/s] [%upulse/L]</li>",
                         flow.CounterGet(FLOW_SENSOR_FRESHWATER),
                         flow.CounterGet(FLOW_SENSOR_FRESHWATER) / flow.DividerGet(FLOW_SENSOR_FRESHWATER),
                         flow.FlowGet(FLOW_SENSOR_FRESHWATER),
                         flow.DividerGet(FLOW_SENSOR_FRESHWATER));
        response->printf("<li>Fs2: [%up] [%uL] [%umL/s] [%upulse/L]</li>",
                         flow.CounterGet(FLOW_SENSOR_NUTRIENTS),
                         flow.CounterGet(FLOW_SENSOR_NUTRIENTS) / flow.DividerGet(FLOW_SENSOR_NUTRIENTS),
                         flow.FlowGet(FLOW_SENSOR_NUTRIENTS),
                         flow.DividerGet(FLOW_SENSOR_NUTRIENTS));
        response->print("<span>Pressure Sensors</span>");
        response->print("<li>");
        response->printf("<button onclick=\"location=\'/pressure?n=1\'\">PS1: measure</button><span> </span>");
        response->printf("[%iUnits] [%u raw/Units] [%i correction]</li>",
                         //pressure.measure(PRES_SENSOR_FRESHWATER),
                         pressure.LastValue(PRES_SENSOR_FRESHWATER),
                         pressure.DividerGet(PRES_SENSOR_FRESHWATER),
                         pressure.OffsetGet(PRES_SENSOR_FRESHWATER));
        response->print("<li>");
        response->printf("<button onclick=\"location=\'/pressure?n=2\'\">PS2: measure</button><span> </span>");
        response->printf("[%iUnits] [%u raw/Units] [%i correction]</li>",
                         //pressure.measure(PRES_SENSOR_NUTRIENTS),
                         pressure.LastValue(PRES_SENSOR_NUTRIENTS),
                         pressure.DividerGet(PRES_SENSOR_NUTRIENTS),
                         pressure.OffsetGet(PRES_SENSOR_NUTRIENTS));
        response->print("<span>Ultrasonic Sensors</span>");

        for (byte i = 0; i < NUM_OF_BARRELS; i++)
        {
            response->print("<li>");
            response->printf("<button onclick=\"location=\'/sonic?n=%u\'\">Sonic %u: measure</button><span> </span>", i, i);
            response->printf("[%iL] [%imm] [%uml/mm] [%umm barrel] [problem:%u]", barrels.SonicCalcLiters(i), barrels.SonicLastMM(i), barrels.SonicMLinMMGet(i), barrels.SonicOffsetGet(i), barrels.ErrorGet(i));
            response->print("</li>");
        }
        response->printf("<li>Sonic liters: [total %i] [usable %i] </li>", barrels.SonicLitersTotal(), barrels.SonicLitersUsable());

        response->print("<span>FMSD:</span>");
        response->print("<form action=\"/man\">");
        response->print("<li><input id=\"barr\" name=\"barr\" value=\"barrel:0-6\"></li>");
        response->print("<li><input id=\"task\" name=\"task\" value=\"fill-1-mix-2-store-3-drain-4\"></li>");
        response->print("<li><input id=\"ammo\" name=\"ammo\" value=\"ammount>0\"></li>");
        response->print("<li>for storing task only:</li>");
        response->print("<li><input id=\"dest\" name=\"dest\" value=\"destination-barr:0-6\"></li>");
        response->print("<li><input type=\"submit\" value=\"Go\">");
        response->print("</form>");

        response->print("</ul>");
        response->print("<button onclick=\"location=location\">reload</button><span> </span>");
        response->print("<button onclick=\"location=\'/reset\'\">reset</button><br>");
        response->printf("<span>uptime: %lli seconds</span><br><br>", esp_timer_get_time() / 1000000);
        response->print("<button onclick=\"location=\'/list\'\">open list filesystem</button>");
        response->print("</body></html>");
        request->send(response);
    });

    server.on("/man", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        if (request->args() > 0)
        { // Arguments were received
            if (request->hasArg("f"))
            {
                int f = request->arg("f").toInt();
                int o = request->arg("o").toInt();
                LOG.printf("setting FillingRelay %i to %i\r\n", f, o);
                expanders.FillingRelay(f, o);
            }
            if (request->hasArg("s"))
            {
                int s = request->arg("s").toInt();
                int o = request->arg("o").toInt();
                LOG.printf("setting StoringRelay %i to %i\r\n", s, o);
                expanders.StoringRelay(s, o);
            }
            if (request->hasArg("d"))
            {
                int d = request->arg("d").toInt();
                int o = request->arg("o").toInt();
                LOG.printf("setting DrainingRelay %i to %i\r\n", d, o);
                expanders.DrainingRelay(d, o);
            }
            if (request->hasArg("rgb"))
            {
                int rgb = request->arg("rgb").toInt();
                LOG.printf("setting setRGBLED to %i\r\n", rgb);
                expanders.setRGBLED(rgb);
            }
            if (request->hasArg("barr"))
            {
                // converting arguments
                int barr = request->arg("barr").toInt();
                int task = request->arg("task").toInt();
                int ammo = request->arg("ammo").toInt();
                int dest = request->arg("dest").toInt();
                // printing message
                LOG.printf("[man]fmsd barr received: barr%i task%i ammo%i dest%i\r\n", barr, task, ammo, dest);
                // checking input is valid
                if (barr > -1 && barr < 7 && task > 0 && task < 5 && ammo > 0 && ammo < 32768 && dest > -1 && dest < 7)
                {
                    // running the task
                    switch (task)
                    {
                        case 1:
                        Fill(barr, ammo);
                        break;
                        case 2:
                        Mix(barr, ammo);
                        break;
                        case 3:
                        Store(barr, dest);
                        break;
                        case 4:
                        Drain(barr, ammo);
                        break;
                    }
                }
                else
                {
                    LOG.println("[E] parameter out of range");
                }
            }
        }
        else
            request->send(200, "text/plain", "no args!");
        request->redirect("/manual");
    });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        request->redirect("/manual");
        ESP.restart();
    });

    server.on("/backup", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        //char buf [4];
        //sprintf (buf, "%03u", Backup());
        //request->send(200, "text/html", buf);
        Backup();
        request->redirect("/list");
    });

    server.on("/restore", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        //char buf [4];
        //sprintf (buf, "%03u", Restore());
        //request->send(200, "text/html", buf);
        Restore();
        request->redirect("/list");
    });

    server.on("/sonic", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        if (request->args() > 0)
        { // Arguments were received
            if (request->hasArg("n"))
            {
                //char buf [6];
                //sprintf (buf, "%05u", barrels.SonicMeasure(request->arg("n").toInt()));
                //request->send(200, "text/html", buf);
                barrels.SonicMeasure(request->arg("n").toInt(), 3, 500); // measure 3 times max 500 ms
                request->redirect("/manual");
            }
        }
        else
            request->send(200, "text/plain", "n parameter missing");
    });

    server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        if (request->args() > 0)
        { // Arguments were received
            if (request->hasArg("n"))
            {
                pressure.measure(request->arg("n").toInt());
                request->redirect("/manual");
            }
        }
        else
            request->send(200, "text/plain", "n parameter missing");
    });

    server.on("/mdns", HTTP_GET, [](AsyncWebServerRequest *request) {
        int mdns = mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        char buf[6];
        sprintf(buf, "mdns:%u", mdns);
        request->send(200, "text/plain", buf);
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        AsyncResponseStream *response = request->beginResponseStream("text/html");
        response->addHeader("Server", "ESP Async Web Server");
        response->printf("<!DOCTYPE html><html><head><title>Webpage at %s</title></head><body>", request->url().c_str());

        response->print("<h2>Hello ");
        response->print(request->client()->remoteIP());
        response->print("</h2>");

        response->print("<h3>General</h3>");
        response->print("<ul>");
        response->printf("<li>Version: HTTP/1.%u</li>", request->version());
        response->printf("<li>Method: %s</li>", request->methodToString());
        response->printf("<li>URL: %s</li>", request->url().c_str());
        response->printf("<li>Host: %s</li>", request->host().c_str());
        response->printf("<li>ContentType: %s</li>", request->contentType().c_str());
        response->printf("<li>ContentLength: %u</li>", request->contentLength());
        response->printf("<li>Multipart: %s</li>", request->multipart() ? "true" : "false");
        response->print("</ul>");

        response->print("<h3>Headers</h3>");
        response->print("<ul>");
        int headers = request->headers();
        for (int i = 0; i < headers; i++)
        {
            AsyncWebHeader *h = request->getHeader(i);
            response->printf("<li>%s: %s</li>", h->name().c_str(), h->value().c_str());
        }
        response->print("</ul>");

        response->print("<h3>Parameters</h3>");
        response->print("<ul>");
        int params = request->params();
        for (int i = 0; i < params; i++)
        {
            AsyncWebParameter *p = request->getParam(i);
            if (p->isFile())
                response->printf("<li>FILE[%s]: %s, size: %u</li>", p->name().c_str(), p->value().c_str(), p->size());
            else if (p->isPost())
                response->printf("<li>POST[%s]: %s</li>", p->name().c_str(), p->value().c_str());
            else
                response->printf("<li>GET[%s]: %s</li>", p->name().c_str(), p->value().c_str());
        }
        response->print("</ul>");

        response->print("</body></html>");
        //send the response last
        request->send(response);
    });

    // Start server
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    server.begin();
    LOG.print(F("-Server init\r\n\r\n"));
} // void setupServer() end

/*-------- NTP code ----------*/
// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address, byte (&packetBuffer)[NTP_PACKET_SIZE])
{
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    UDP.beginPacket(address, NTP_PORT);
    UDP.write(packetBuffer, NTP_PACKET_SIZE);
    UDP.endPacket();
}

time_t getNtpTime()
{
    isTimeSync = true;                  // prevent retrigger untill done
    byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
    IPAddress ntpServerIP;              // NTP server's ip address
    while (UDP.parsePacket() > 0)
        ;                                       // discard any previously received packets
    WiFi.hostByName(NTP_HOSTNAME, ntpServerIP); // get a random server from the pool
#ifdef DEBUG
    LOG.print("NTP time request via ");
    LOG.print(ntpServerIP);
#endif
    sendNTPpacket(ntpServerIP, packetBuffer);
    uint32_t beginWait = millis();
    while (millis() - beginWait < NTP_TIMEOUT)
    {
        byte size = UDP.parsePacket();
        if (size >= NTP_PACKET_SIZE)
        {
            LOG.println(" Received NTP Response");
            UDP.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer
            unsigned long secsSince1900;
            // convert four bytes starting at location 40 to a long integer
            secsSince1900 = (unsigned long)packetBuffer[40] << 24;
            secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
            secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
            secsSince1900 |= (unsigned long)packetBuffer[43];
            uint16_t DTS = 0;                                              //mySettings.DTS?3600UL:0;
            uint16_t timeZone = 3;                                         // temporary
            return secsSince1900 - 2208988800UL + DTS + timeZone * 3600UL; // added DTS
        }
    }
    LOG.println("No NTP Response :-(");
    isTimeSync = false; // if failed - set back to false
    return 0;           // return 0 if unable to get the time
}

//RTC module
/*
RtcDS3231<TwoWire> Rtc(Wire);



void setupRTC (){
    Rtc.Begin();
    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    if (!Rtc.IsDateTimeValid()) {
        if (Rtc.LastError() != 0) // communication error
        {
              LOG.println(Rtc.LastError());
        }
        else // RTC lost confidence in the DateTime
            Rtc.SetDateTime(compiled);
    }
    if (!Rtc.GetIsRunning()) 
        Rtc.SetIsRunning(true);
    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled)
        Rtc.SetDateTime(compiled); // sets time to PC time if needed
    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 
}

//temporary test
void testPrintRTC() 
{
    //RtcDateTime now = Rtc.GetDateTime();
    //printDateTime(now);
    LOG.println(getDateTimeNow());
	RtcTemperature temp = Rtc.GetTemperature();
    LOG.print(temp.AsFloatDegC());
    LOG.println(" *C");
}


//void printDateTime(const RtcDateTime& dt)

// Returns Date-Time string
String getDateTimeNow(){
    RtcDateTime currentTime = Rtc.GetDateTime();    //get the time from the RTC
    char str[20];   //declare a string as an array of chars
    sprintf(str, "%d/%d/%d %d:%d:%d",     //%d allows to print an integer to the string
        currentTime.Year(),currentTime.Month(),currentTime.Day(),
        currentTime.Hour(),currentTime.Minute(),currentTime.Second());
    return (str);
}



alarm - needs an interrupt pin connected to RTC square wave / alarm pin (SQW) 
https://github.com/Makuna/Rtc/wiki/RtcDS3231-AlarmOne
https://techtutorialsx.com/2017/02/04/esp8266-ds3231-alarm-when-seconds-match/


*/

void LoadStructs()
{
    if (isSD)
        Restore(); // in case something is missing in the SD
    LOG.printf("Loading system from %s\r\n", isSD ? "SD" : "SPIFFS");
    if (disk->exists("/SysState.bin"))
        SystemState.LoadSD();
    else
    {
        LOG.println("/SysState.bin\tnot exist");
    }

    if (disk->exists("/Pressure.bin"))
        pressure.LoadSD();
    else
    {
        LOG.println("/Pressure.bin\tnot exist");
    }

    if (disk->exists("/Flow.bin"))
        flow.LoadSD();
    else
    {
        LOG.println("/Flow.bin\tnot exist");
    }

    if (disk->exists("/Barrels.bin"))
        barrels.LoadSD();
    else
    {
        LOG.println("/Barrels.bin\tnot exist");
    }
}

// reimplement later to save on demand only what
// this one will be used on system stop? save all...?
// should run on a separate thread? or ommit the waits for no flow?
// can also stop all relays (if not already stopped?) expanders.Protect
//and undo protect when finished.
void SaveStructs()
{
    LOG.println(F("save-all trigerred"));
    if (!isSaving)
    { // prevent concurrent saving
        isSaving = true;
        if (!isSD)
        {
            LOG.println(F("SD card not ready - trying to restart"));
            SD.end();
            if (SD.begin())
            {
                LOG.println(F("SD card OK"));
                disk = &SD;
                isSD = true;
            }
            else
            {
                LOG.println(F("SD failed. resorting to SPIFFS"));
                SendSMS("SD card unable to save");
            }
        }
        LOG.println(F("waiting for no flow.."));
        while (flow.FlowGet(FLOW_SENSOR_FRESHWATER))
            ;
        while (flow.FlowGet(FLOW_SENSOR_NUTRIENTS))
            ;
        LOG.println(F("no flow ok - saving..."));
        SystemState.SaveSD();

        // reimplement to save ondemand?
        barrels.SaveSD();
        pressure.SaveSD();
        flow.SaveSD();
        isSaving = false;
        LOG.println(F("all save finished."));
    }
}

/* initial sequnence */
void setup()
{
    // initiate UART0 - serial output
    Serial.begin(115200);

    // initialize UART2 - serial MUX
    Serial2.begin(9600, SERIAL_8N1);

    //WiFiManager Local intialization. Once its business is done, there is no need to keep it around
    AsyncWiFiManager wifiManager(&server, &dns);
    //wifiManager.resetSettings();
    wifiManager.setConfigPortalTimeout(180);  // 3 minutes
    wifiManager.autoConnect("AutoConnectAP"); // will stop here if no wifi connected
    LOG.printf("ESSID: %s\r\n", WiFi.SSID().c_str());

    //initiate expander ports - I2C wire
    expanders.Init();

    // initiate SD Card - SPI bus + SPIFFS
    initStorage();

    // load structs from SD card
    LoadStructs();

    pressure.setSensor(PRES_SENSOR_FRESHWATER, PRESSUR_1_PIN);
    pressure.setSensor(PRES_SENSOR_NUTRIENTS, PRESSUR_2_PIN);

    flow.begin();
    flow.CounterReset(FLOW_SENSOR_FRESHWATER);
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS);

    //bug? testing..
    expanders.UnlockMUX();

    // initialize uart2 SIM800L modem at MUX port 7?
    modemInit();

    // setup NTP

    //start Web Server
    setupServer();
    ArduinoOTA.setHostname("barrels");
    ArduinoOTA.begin();
    //mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    MDNS.addService("http", "tcp", 80); // add mDNS http port
    // attach interrupts for flow sensors

    // Create tasks
    /*
    xTaskCreatePinnedToCore(myTask, "loop1", 4096, (void *)1, 1, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(myTask, "loop2", 4096, (void *)2, 1, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(myTask, "loop3", 4096, (void *)3, 1, NULL, ARDUINO_RUNNING_CORE);

    void myTask(void *pvParameters) {
        int taskno = (int)pvParameters;
        int sleeptime;
        while (1) {
            sleeptime = (int)(esp_random()&0x0F);
            LOG.println(String("Task ")+String(taskno)+String(" ")+String(sleeptime));
            delay(500+sleeptime*100);
        }
    }

    // disable loop watchdog - working with tasks only?
    void loop() {
    // nope, do nothing here
    vTaskDelay(portMAX_DELAY); // wait as much as posible ...
    }

    */

    //SendSMS("System Started");
}

void loop()
{
    ArduinoOTA.handle();
}