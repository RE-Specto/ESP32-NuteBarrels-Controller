/*
Copyright 2019-2021 Evgeni Vi
more info and license - soon
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


#define MUX_UNLOCKED 255 //valid channels are 0-15

#define LOG Serial.printf("time[%06lli]line[%04i]func[%s] ", esp_timer_get_time() / 1000000, __LINE__, __FUNCTION__);Serial   //SerialAndTelnet or file
#define SYNC_INTERVAL 600 // NTP sync - in seconds
#define NTP_PACKET_SIZE 48
// NTP time is in the first 48 bytes of message
#define NTP_HOSTNAME "pool.ntp.org"
#define NTP_PORT 123
#define NTP_TIMEOUT 5000
#define LOCAL_UDP_PORT 8888 // local port to listen for UDP packets

// load Object from Storage to Struct
#define NUM_OF_BARRELS 6
#define POOLS 6 // barrels from 0 to 5, pools out at number 6

//Flow sensor Pin declarations
#define FLOW_1_PIN 25
#define FLOW_2_PIN 26
//#define FLOW_3_PIN 27

//Pressure sensor Pin declarations
#define PRESSUR_1_PIN 36
#define PRESSUR_2_PIN 34
//#define PRESSUR_3_PIN 35

// Start-Stop Buttons
#define START_PIN 4
#define STOP_PIN 0

// delay for SMS error report
#define REPORT_DELAY 5000

#define DEBUG
//#define DEBUG_SD


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
TaskHandle_t loop1; // fmsTask handle

//function declarations:
bool Load(const char *fname, byte *stru_p, uint16_t len);
bool Save(const char *fname, byte *stru_p, uint16_t len);
void SaveStructs();
void IRAM_ATTR FlowSensor1Interrupt();
void IRAM_ATTR FlowSensor2Interrupt();
void IRAM_ATTR StartButtonInterrupt();
void IRAM_ATTR StopButtonInterrupt();
void SendSMS(const char *message, byte item = 0xFF);
void Fill(byte barrel, uint16_t requirement);
void Mix(byte barrel);
void Store(byte barrel, byte target);
void Drain(byte barrel, uint16_t requirement);

//structs
struct myST
{
    //bit field
    //00NFMSEX
    // N = maNual mode on - overrides stopped
    // F = Filling task on
    // M = Mixing task on
    // S = Storing task on
    // E = Emptying task on
    // X = stopped status on
    // 00NFMSEX
    byte _state_now = 17; // 00010001 - filling + waiting for nutes - initial system state
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
    //9-256-
    //A-512-
    //B-1024-
    //C-2048-
    //D-4096-
    //E-8192-
    //F-16384-
    uint16_t _error_now = 0;
    uint16_t _error_before = 0;
    // fmsd
    uint16_t _fill_req = 500;            // total liters to fill 
    uint16_t _mix_req = 3;               // total minutes to mix
    uint16_t _mix_timer = _mix_req;   // minutes to mix this time
    uint16_t _drain_req = 0;             // how many liters to drain
    uint32_t _drain_total = 0;                 // how many liters totally drained
    byte _filling_barrel = 0;                    // mixer barrel
    byte _storing_barrel = NUM_OF_BARRELS - 1;   // last barrel (NUM_OF_BARRELS -1 cause we start from zero)
    // 0 cancel, 1 fill, 2 mix, 3 store, 4 drain
    byte _manual_task = 0;
    byte _manual_src = 0;
    byte _manual_dest = 0;
    uint16_t _manual_ammo = 0;
};

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
    uint16_t _VolumeMax = 500; // for flow and sonic sensors
    // calibrate by filling 100L
    // also can be calculated by barrel diameter
    uint16_t _SonicMLinMM = 1130; // mililiters in 1 milimeter
    // calibrate by dry barrel height
    uint16_t _SonicOffset = 1000; // mm to barrel's full point
};

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
    bool _enabled = false;
};

struct myFSENS
{
    volatile uint32_t counter = 0;
    uint16_t conversion_divider = 450;
    volatile uint32_t flow = 0;
    volatile uint32_t lastMilis = 0;
};

//classes
class ExpClass
{
private:
    byte _muxLock = MUX_UNLOCKED; // unlocked initially
public:
    void Init();
    void LockMUX(byte address);
    byte GetMUX();
    void UnlockMUX();
    void setRGBLED(byte address);
    void Protect(bool state);
    void FillingRelay(byte address, bool state);
    byte FillingRelayGet(byte address);
    void StoringRelay(byte address, bool state);
    byte StoringRelayGet(byte address);
    void DrainingRelay(byte address, bool state);
    byte DrainingRelayGet(byte address);
    void Pump(bool state);
} expanders; // initiated globally

class STClass
{
private:
    myST myState;
public:
    bool LoadSD();
    bool SaveSD();
    byte state_get();
    void IRAM_ATTR state_set(byte mask);
    void IRAM_ATTR state_unset(uint16_t mask);
    void state_save();
    void state_load();
    bool state_check(byte mask);
    void state_print();
    bool state_change_check();
    uint16_t error_get();
    bool error_check(uint16_t error);
    void error_set(uint16_t error);
    void error_unset(uint16_t error);
    uint16_t error_getnew();
    void error_reported();
    void begin();
    uint16_t fill_req_get();
    void fill_req_set(uint16_t req);
    byte fill_barrel();
    void fill_barrel_set(byte barrel);
    uint16_t mix_req_get();
    void mix_req_set(uint16_t req);
    uint16_t mix_needed();
    void mix_less();
    void mix_reset();
    byte store_barrel();
    void store_barrel_set(byte barrel);
    void store_barrel_up();
    void store_barrel_down();
    void drain_req_set(uint16_t req);
    uint16_t drain_needed();
    void drain_substract(byte barrel);
    void manual_task_set(byte task, byte src, byte dest, uint16_t ammo);
    void IRAM_ATTR manual_task_reset();
    byte manual_task();
    byte manual_from();
    byte manual_to();
    uint16_t manual_howmuch();

} System;

class BARRClass
{
private:
    myBR myBarrel[8];
public:
    bool LoadSD();
    bool SaveSD();
    byte ErrorGet(byte barrel);
    bool ErrorCheck(byte barrel, byte mask);
    void ErrorSet(byte barrel, byte mask);
    void ErrorUnset(byte barrel, byte mask);
    void ErrorReset(byte barrel);
    uint16_t FreshGet(byte barrel);
    uint16_t NutriGet(byte barrel);
    void NutriLess(byte barrel, uint32_t liters);
    void FreshwaterFillCalc(byte barrel);
    float ConcentrationTotal(byte barrel);
    void ConcentrationRecalc(byte barrel);
    bool isFillTargetReached(byte barrel, byte type, uint16_t target);
    void NutrientsTransferCalc(byte from, byte to);
    uint16_t VolumeMax(byte barrel);
    uint16_t VolumeMin(byte barrel);
    void SonicMeasure(byte barrel, byte measure = 10, uint16_t timeLeft = 1000, byte retryLeft = 5);
    uint16_t SonicLastMM(byte barrel);
    int16_t SonicLastMMfromEmpty(byte barrel);
    uint16_t SonicOffsetGet(byte barrel);
    uint16_t SonicMLinMMGet(byte barrel);
    void SonicOffsetSet(byte barrel, uint16_t offs);
    void SonicMLinMMSet(byte barrel, uint16_t coef);
    int16_t SonicCalcLiters(byte barrel);
    int8_t BarrelPercents(byte barrel);
    int16_t SonicLitersTotal();
    int16_t SonicLitersUsable();
    bool isDry(byte barrel);
    bool isEmpty(byte barrel);
    bool isFull(byte barrel);
} barrels;

class FSClass
{
private:
    myFSENS fsensor[2]; // two sensors
public:
    uint16_t FlowGet(byte sens);
    void IRAM_ATTR CounterInc(byte sens);
    uint32_t CounterGet(byte sens);
    void CounterSubtract(byte sens, uint32_t value);
    void CounterReset(uint_fast8_t sens);
    uint16_t DividerGet(byte sens);
    void DividerSet(byte sens, uint16_t div);
    bool LoadSD();
    bool SaveSD();
    void begin();
} flow;

class PSClass
{
private:
    myPS psensor[2]; // two sensor
public:
    void setSensor(byte sens, byte sensorPin);
    byte DividerGet(byte sens);
    void DividerSet(byte sens, byte div);
    int16_t OffsetGet(byte sens);
    void OffsetSet(byte sens, int16_t offs);
    byte MaxGet(byte sens);
    void MaxSet(byte sens, byte max);
    byte MinGet(byte sens);
    void MinSet(byte sens, byte min);
    bool LoadSD();
    bool SaveSD();
    int16_t LastValue(byte sens);
    int16_t measure(byte sens);
    byte ErrorGet(byte sens);
    void ErrorReset(byte sens);
    bool Enabled(byte sens);
    void Enable(byte sens);
    void Disable(byte sens);
} pressure; 

/*-------- Filesystem Begin ----------*/
// SD Card and SPIFFS
void initStorage()
{
    byte retry = 5; // 5 times for SD
    while (retry)
    {
        if (SD.begin())
        {
            #ifdef DEBUG_SD
            LOG.println(F("SD card OK"));
            #endif
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
            #ifdef DEBUG_SD
            LOG.println(F("SPIFFS OK"));
            #endif
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

// restore only files missing on SD card. do not override existing files
byte Restore()
{
    #ifdef DEBUG_SD
    LOG.println(F("Restoring missing files from SPIFFS to SD"));
    #endif
    File dir = SPIFFS.open("/");
    File file = dir.openNextFile();
    size_t len = 0; // file chunk lenght at the buffer
    byte counter = 0;
    while (file)
    {
        //file.name() file.size());
        if (SD.exists(file.name()))
        {
            #ifdef DEBUG_SD
            LOG.printf("file %s already exist on SD\r\n", file.name());
            #endif
        }
        else if (!file.size())
        {
            #ifdef DEBUG_SD
            LOG.printf("skipping empty file %s\r\n", file.name());
            #endif
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
    #ifdef DEBUG_SD
    LOG.printf("Restore finished. %u files copied\r\n", counter);
    #endif
    return counter;
}
/*-------- Filesystem END ----------*/

/*-------- Expanders Begin ----------*/
// declare I/O Expander ports and constants

// MCP23017 with pin settings
MCP23017 expander1(0); // Base Address + 0: 0x20
MCP23017 expander2(1); // Base Address + 1: 0x21

void ExpClass::Init()
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
void ExpClass::LockMUX(byte address)
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
byte ExpClass::GetMUX() { return _muxLock; }

// very important to run this every time you ended up business with LockMUX
void ExpClass::UnlockMUX()
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
void ExpClass::setRGBLED(byte address)
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
void ExpClass::Protect(bool state)
{
    // triggers last relay in each relay board to disconnect 12v line
    expander1.getPin(15).setValue(!state); // filling relay protect pin
    expander2.getPin(7).setValue(!state);  // storing relay protect pin
    expander2.getPin(15).setValue(!state); // draining relay protect pin
    expander1.write();
    expander2.write();
    LOG.printf("!! Protection %s\r\n", state ? "On!" : "Off");
}

// triggers filling relay
void ExpClass::FillingRelay(byte address, bool state)
{
    //offset of 8 - expander 0x20 pins b0-b7
    expander1.getPin(address + 8).setValue(!state);
    expander1.write();
}

// get filling relay state
byte ExpClass::FillingRelayGet(byte address)
{
    //offset of 8 - expander 0x20 pins b0-b7
    //expander1.read();
    return !expander1.getPin(address + 8).getValue();
}

// triggers storing relay
void ExpClass::StoringRelay(byte address, bool state)
{
    //offset of 0 - expander 0x21 pins a0-a7
    expander2.getPin(address).setValue(!state);
    expander2.write();
}

// get storing relay state
byte ExpClass::StoringRelayGet(byte address)
{
    //offset of 8 - expander 0x20 pins b0-b7
    //expander1.read();
    return !expander2.getPin(address).getValue();
}

// triggers draining relay
void ExpClass::DrainingRelay(byte address, bool state)
{
    //offset of 8 - expander 0x21 pins b0-b7
    expander2.getPin(address + 8).setValue(!state);
    expander2.write();
}

// get draining relay state
byte ExpClass::DrainingRelayGet(byte address)
{
    //offset of 8 - expander 0x20 pins b0-b7
    //expander1.read();
    return !expander2.getPin(address + 8).getValue();
}

// starts/stops the pump
void ExpClass::Pump(bool state)
{
    expander1.getPin(14).setValue(!state); // filling relay pump pin
    expander1.write();
}
/*-------- Expanders END ----------*/

/*-------- Modem Begin ----------*/
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

    expanders.LockMUX(7);                          // modem is at port 7
    Alarm.delay(10);                              // wait until expander + mux did their job
    Serial2.println("AT+CMGS=\"+972524373724\""); //change ZZ with country code and xxxxxxxxxxx with phone number to sms
    Serial2.print(message);                       //text content
    if (item != 0xFF)
        Serial2.print(item);
    Serial2.write(26); // send ctrl+z end of message
    //Serial2.print((char)26); // if the above won't work - thy this one instead
    expanders.UnlockMUX();
    //Serial.println(); // add newline after sendsms for log readability
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

void modemInit()
{
    LOG.println("-modem init");
    expanders.LockMUX(7); // modem is at port 7
    Alarm.delay(10);     // wait until expander + mux did their job
    // Serial2.begin(9600, SERIAL_8N1); // already done in main
    Serial2.println("AT");        //Once the handshake test is successful, it will back to OK
    Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
    expanders.UnlockMUX();        // Must unlock after every use!!
}
/*-------- Modem END ----------*/

/*-------- System Begin ----------*/
bool STClass::LoadSD() { return Load("/SysState.bin", (byte *)&myState, sizeof(myState)); }

bool STClass::SaveSD() { return Save("/SysState.bin", (byte *)&myState, sizeof(myState)); }

// returns system state raw integer value (0=uninitiated)
byte STClass::state_get() { return myState._state_now; }

// set state using mask (FILLING_STALE,STORING_STATE....)
void IRAM_ATTR STClass::state_set(byte mask) { myState._state_now |= mask; }

// unset mask from state  (FILLING_STALE,STORING_STATE....)
void IRAM_ATTR STClass::state_unset(uint16_t mask) { myState._state_now &= ~mask; }

// preserve prevoius state
void STClass::state_save() { myState._state_before = myState._state_now; }

// restore previous state
void STClass::state_load() { myState._state_now = myState._state_before; }

// returns true if state have "mask-bit" state on. ex: state_check(MIXING_STATE);
bool STClass::state_check(byte mask) { return myState._state_now & mask; }

void STClass::state_print()
{
    LOG.printf("system state:%u ", state_get());
    // keep it simple
    if (state_check(MANUAL_STATE))
        Serial.print(F("MANUAL "));
    if (state_check(FILLING_STATE))
        Serial.print(F("FILLING "));
    if (state_check(MIXING_STATE))
        Serial.print(F("MIXING "));
    if (state_check(STORING_STATE))
        Serial.print(F("STORING "));
    if (state_check(DRAINIG_STATE))
        Serial.print(F("DRAINIG "));
    if (state_check(STOPPED_STATE))
        Serial.print(F("STOPPED "));
    Serial.println();
}

// detects and prints system state change
bool STClass::state_change_check()
{
    if (myState._state_now != myState._state_before)
    {
        state_save();
        state_print();
        return true;
    }
    return false;
}

// returns error state
uint16_t STClass::error_get() { return myState._error_now; }

// returns true if error have "mask-bit" state on. ex: error_check(ERR_WATER_NOFLOW);
bool STClass::error_check(uint16_t error) { return myState._error_now & error; }

// set error using mask
void STClass::error_set(uint16_t error) { myState._error_now |= error; }

// unset error using mask
void STClass::error_unset(uint16_t error) { myState._error_now &= ~error; }

//returns the difference between current and last error states
// !! need to reimplement to return either the new error position
// so I dont have to use uint16_t for return
// or either if two errors should be reported at once - a while loop that solves them one by one
uint16_t STClass::error_getnew()
{
    // bitwise - diff between "before" and "now",
    // compare with "now" to extract new bits only
    return (myState._error_now ^ myState._error_before) & myState._error_now;
}

void STClass::error_reported() { myState._error_before = myState._error_now; }

void STClass::begin()
{
    LOG.printf("-System: init start stop at pins %u, %u\r\n", START_PIN, STOP_PIN);
    pinMode(START_PIN, INPUT_PULLUP);
    // pinMode(STOP_PIN, INPUT_PULLUP); // already pulled up by hardware
    attachInterrupt(digitalPinToInterrupt(START_PIN), StartButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(STOP_PIN), StopButtonInterrupt, FALLING);
}

uint16_t STClass::fill_req_get() {return myState._fill_req;}

void STClass::fill_req_set(uint16_t req) {myState._fill_req = req;}

byte STClass::fill_barrel() {return myState._filling_barrel;}

void STClass::fill_barrel_set(byte barrel) {myState._filling_barrel = barrel;}

uint16_t STClass::mix_req_get() {return myState._mix_req;}

void STClass::mix_req_set(uint16_t req) {myState._mix_req = req;}

// returns true if still need to mix 
uint16_t STClass::mix_needed() {return myState._mix_timer;}

// decreases mix timer
void STClass::mix_less() {myState._mix_timer--;}

// returns mix timer to default value
void STClass::mix_reset() {myState._mix_timer = myState._mix_req;}

byte STClass::store_barrel() {return myState._storing_barrel;}

void STClass::store_barrel_set(byte barrel) {myState._storing_barrel = barrel;}

void STClass::store_barrel_up() 
{
    myState._storing_barrel++;
    LOG.printf("Storing barrel inc, now barrel#%u\r\n", myState._storing_barrel);
}

void STClass::store_barrel_down()
{
    myState._storing_barrel--;
    LOG.printf("Storing barrel dec, now barrel#%u\r\n", myState._storing_barrel);
}

void STClass::drain_req_set(uint16_t req) {myState._drain_req = req;}

// returns true if still need to drain
uint16_t STClass::drain_needed() {return myState._drain_req;}

// substract flow sensor counted liters from drain requirement
// increase total counted drain
// substract counted ammount from flow counter
void STClass::drain_substract(byte barrel)
{
    // flow counter may be increased during calculation 
    // because flowsensor works on interrupts
    // so capturing only the current state for count
    uint32_t liters = flow.CounterGet(FLOW_SENSOR_NUTRIENTS) / flow.DividerGet(FLOW_SENSOR_NUTRIENTS);
    if (myState._drain_req) // prevent integer overflow
        myState._drain_req -= liters;
    if (myState._drain_total<UINT32_MAX) // prevent integer overflow
        myState._drain_total += liters;
    barrels.NutriLess(barrel, liters);
    // decrement flow counter by "counted so far" pulses (liters times divider)
    flow.CounterSubtract(FLOW_SENSOR_NUTRIENTS, liters * flow.DividerGet(FLOW_SENSOR_NUTRIENTS));
}

void STClass::manual_task_set(byte task, byte src, byte dest, uint16_t ammo)
{
    myState._manual_task=task;
    myState._manual_src=src;
    myState._manual_dest=dest;
    myState._manual_ammo=ammo;
}

void IRAM_ATTR STClass::manual_task_reset(){myState._manual_task=0;}

byte STClass::manual_task() {return myState._manual_task;}

byte STClass::manual_from() {return myState._manual_src;}

byte STClass::manual_to() {return myState._manual_dest;}

uint16_t STClass::manual_howmuch() {return myState._manual_ammo;}

void IRAM_ATTR StartButtonInterrupt()
{
    portENTER_CRITICAL_ISR(&mux);
    System.state_unset(STOPPED_STATE);
    System.manual_task_reset();
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR StopButtonInterrupt()
{
    portENTER_CRITICAL_ISR(&mux);
    System.state_set(STOPPED_STATE);
    System.manual_task_reset();
    portEXIT_CRITICAL_ISR(&mux);
}

/*-------- System END ----------*/

// deprecate?? send sms on demand in each function?
//what if need to send sms in middle of mux lock? sonic measurement?
// Task 0 - Error reporting task
void errorReportTask()
{
    // while there is no new errors
    while (!System.error_getnew())
    {
        Alarm.delay(REPORT_DELAY); //wait - endless loop untill error status changes
    }

    // goes here if there is an error to report
    uint16_t newerrors = System.error_getnew();
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
    System.error_reported();
}

/*-------- FlowSensor Begin ----------*/
// flow Sensors Pin Declarations
// and Interrupt routines
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

// returns flow in mililiters per second for sensor No(sens)
uint16_t FSClass::FlowGet(byte sens)
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    // measurement older than 1 second means no flow
    if ((millis() - fsensor[sens].lastMilis) > 1000)
        return 0;
    return ((1000000 / (fsensor[sens].flow ? fsensor[sens].flow : 1)) / fsensor[sens].conversion_divider); // in mililiters per second
}                                                                                                          // Guru Meditation Error: Core  1 panic'ed (IntegerDivideByZero). Exception was unhandled.

// executed by interrupt to increase counter
void IRAM_ATTR FSClass::CounterInc(byte sens)
{
    fsensor[sens].counter++;
    fsensor[sens].flow = millis() - fsensor[sens].lastMilis; // interval in miliseconds from the last call
    fsensor[sens].lastMilis = millis();
}

// get raw flow counter pulses
uint32_t FSClass::CounterGet(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return fsensor[sens].counter; 
}

// Subtracts count from sensor (after we applied the count to target barrel)
void FSClass::CounterSubtract(byte sens, uint32_t value)
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
void FSClass::CounterReset(uint_fast8_t sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    fsensor[sens].counter = 0; 
}

// returns pulses to liter
uint16_t FSClass::DividerGet(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return fsensor[sens].conversion_divider; 
}

// set pulses to liter (sensor number, divider)
void FSClass::DividerSet(byte sens, uint16_t div) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    fsensor[sens].conversion_divider = div; 
}

bool FSClass::LoadSD() { return Load("/Flow.bin", (byte *)&fsensor, sizeof(fsensor)); }

bool FSClass::SaveSD() { return Save("/Flow.bin", (byte *)&fsensor, sizeof(fsensor)); }

// attach interrupts
void FSClass::begin()
{
    LOG.printf("-Flow: init sensors at pins %u, %u\r\n", FLOW_1_PIN, FLOW_2_PIN);
    pinMode(FLOW_1_PIN, INPUT_PULLUP);
    pinMode(FLOW_2_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_1_PIN), FlowSensor1Interrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(FLOW_2_PIN), FlowSensor2Interrupt, FALLING);
}

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
/*-------- FlowSensor END ----------*/

/*-------- PressureSensor Begin ----------*/
// Pressure Sensors Pin Declarations
// and Analog reads
// task pressure sensors - stops pumps on overpressure

// init the sensor
void PSClass::setSensor(byte sens, byte sensorPin)
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    psensor[sens]._sensorPin = sensorPin;
    pinMode(sensorPin, INPUT); // initialize analog pin for the sensor
}

byte PSClass::DividerGet(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return psensor[sens]._divider; 
}

void PSClass::DividerSet(byte sens, byte div) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    psensor[sens]._divider = div; 
}

int16_t PSClass::OffsetGet(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return psensor[sens]._offset; 
}

void PSClass::OffsetSet(byte sens, int16_t offs) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    psensor[sens]._offset = offs; 
}

byte PSClass::MaxGet(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return psensor[sens]._max_pressure; 
}

void PSClass::MaxSet(byte sens, byte max) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    psensor[sens]._max_pressure = max; 
}

byte PSClass::MinGet(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return psensor[sens]._min_pressure; 
}

void PSClass::MinSet(byte sens, byte min) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    psensor[sens]._min_pressure = min; 
}

bool PSClass::LoadSD() { return Load("/Pressure.bin", (byte *)&psensor, sizeof(psensor)); }

bool PSClass::SaveSD() { return Save("/Pressure.bin", (byte *)&psensor, sizeof(psensor)); }

// returns last value measured by pressure sensor (sens)
int16_t PSClass::LastValue(byte sens)
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return psensor[sens]._last_pressure;
}

// read sensor (sens) - converts analog value to pressure
int16_t PSClass::measure(byte sens)
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

    //4  short circuit
    if (pres > 255)
    {
        // stop pump
        expanders.Pump(false);
        Alarm.delay(100);
        expanders.Protect(true);
        // set error
        if (p->_ErrorState != PRESSURE_SHORTCIRCUIT)
        {
            p->_ErrorState = PRESSURE_SHORTCIRCUIT;
            LOG.printf("[E] sensor %u short circuit\r\n", sens + 1);
            SendSMS("short circuit @pressure sensor", sens + 1);
        }
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
        if (p->_ErrorState != PRESSURE_DISCONNECT)
        {
            p->_ErrorState = PRESSURE_DISCONNECT;
            LOG.printf("[E] sensor %u disconnected\r\n", sens + 1);
            SendSMS("lost connection @pressure sensor", sens + 1);
        }
        return pres; // exits here
    }

    //2  overpressure
    if (pres > p->_max_pressure && pres < 255)
    {
        // stop pump
        expanders.Pump(false);
        Alarm.delay(100);
        expanders.Protect(true);
        // set error
        if (p->_ErrorState != PRESSURE_OVERPRESSURE)
        {
            p->_ErrorState = PRESSURE_OVERPRESSURE;
            LOG.printf("[E] sensor %u overpressure\r\n", sens + 1);
            SendSMS("Overpressure @pressure sensor", sens + 1);
        }
        return pres; // exits here
    }

    //1  no pressure
    if (pres < p->_min_pressure)
        if (p->_ErrorState == PRESSURE_NORMAL)
            p->_ErrorState = PRESSURE_NOPRESSURE;

    //0  normal pressure range is here
    if (pres > p->_min_pressure && pres < p->_max_pressure)
        if (p->_ErrorState == PRESSURE_NOPRESSURE)
            p->_ErrorState = PRESSURE_NORMAL;

    // set system-wide pressure error if critical error
    if (p->_ErrorState > 1)
    {
        expanders.setRGBLED(LED_RED);
        if (sens == 0)
            System.error_set(ERR_WATER_PRESSURE);
        else
            System.error_set(ERR_NUTRI_PRESSURE);
    }
    return pres;
}

// error handling
//0  normal pressure
//1  no pressure
//2  overpressure
//3  disconnected
//4  short circuit
byte PSClass::ErrorGet(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return psensor[sens]._ErrorState; 
}

// disable protect
// unset system error
// remeasure
void PSClass::ErrorReset(byte sens)
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    expanders.Protect(false);
    psensor[sens]._ErrorState = 0;
    if (!sens) // sensor-0 freshwater
        System.error_unset(ERR_WATER_PRESSURE);
    else // sensor-1 nutrients
        System.error_unset(ERR_NUTRI_PRESSURE);
    // back to one - cause we calling function that also counts from one
    sens++;
    measure(sens);
}

bool PSClass::Enabled(byte sens) { return psensor->_enabled; }

void PSClass::Enable(byte sens) { psensor[sens]._enabled = true; }

void PSClass::Disable(byte sens) { psensor[sens]._enabled = false; }

/*-------- PressureSensor END ----------*/

/*-------- Barrels Begin ----------*/
bool BARRClass::LoadSD() { return Load("/Barrels.bin", (byte *)&myBarrel, sizeof(myBarrel)); }

bool BARRClass::SaveSD() { return Save("/Barrels.bin", (byte *)&myBarrel, sizeof(myBarrel)); }

// error handling
byte BARRClass::ErrorGet(byte barrel) { return myBarrel[barrel]._ErrorState; }
bool BARRClass::ErrorCheck(byte barrel, byte mask) { return myBarrel[barrel]._ErrorState & mask; }
void BARRClass::ErrorSet(byte barrel, byte mask)
{
    myBarrel[barrel]._ErrorState |= mask;
    LOG.printf("[E] Barr%u:e%u\r\n", barrel, mask);
}
void BARRClass::ErrorUnset(byte barrel, byte mask) { myBarrel[barrel]._ErrorState &= ~mask; }
void BARRClass::ErrorReset(byte barrel) { myBarrel[barrel]._ErrorState = 0; }

// freshwater counted by flow for (barrel)
uint16_t BARRClass::FreshGet(byte barrel) { return myBarrel[barrel]._VolumeFreshwater; }

// nutrients counted by flow for (barrel)
uint16_t BARRClass::NutriGet(byte barrel) { return myBarrel[barrel]._VolumeNutrients; }

// decrease barrel by drained ammount
void BARRClass::NutriLess(byte barrel, uint32_t liters)
{
    myBarrel[barrel]._VolumeNutrients -= liters;
}

// add fresh flow couter to barrel, then Subtract it from flowsensor
void BARRClass::FreshwaterFillCalc(byte barrel)
{
    myBR *b = &myBarrel[barrel];
    if (b->_VolumeFreshwaterLast != b->_VolumeFreshwater) // safeguard
        b->_VolumeFreshwater = b->_VolumeFreshwaterLast;

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
float BARRClass::ConcentrationTotal(byte barrel)
{
    myBR *b = &myBarrel[barrel];
    if (!b->_VolumeNutrients && !b->_VolumeFreshwater)
        return 0; // empty barrels - prevent Divide By Zero!
    else
        return b->_Concentraion * b->_VolumeNutrients / (b->_VolumeNutrients + b->_VolumeFreshwater); // mix
}

// calc new concentration:
// counts all as nutrinets after calculation, at concentration X
void BARRClass::ConcentrationRecalc(byte barrel)
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
bool BARRClass::isFillTargetReached(byte barrel, byte type, uint16_t target)
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
void BARRClass::NutrientsTransferCalc(byte from, byte to)
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

    if (b->_VolumeNutrientsLast != b->_VolumeNutrients)
        b->_VolumeNutrients = b->_VolumeNutrientsLast;

    if (a->_ConcentraionLast != a->_Concentraion)
        a->_Concentraion = a->_ConcentraionLast;

    if (b->_ConcentraionLast != b->_Concentraion)
        b->_Concentraion = b->_ConcentraionLast;

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

uint16_t BARRClass::VolumeMax(byte barrel)
{
    return myBarrel[barrel]._VolumeMax;
}

uint16_t BARRClass::VolumeMin(byte barrel)
{
    return myBarrel[barrel]._VolumeMin;
}

// contact the sensor via UART, measure, return distance in mm
void BARRClass::SonicMeasure(byte barrel, byte measure, uint16_t timeLeft, byte retryLeft)
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
    expanders.LockMUX(barrel);
    delay(1);
    for (byte x = 0; x < measure && retryLeft && timeLeft;)
    {
        // send data so sonic will reply
        Serial2.write(0x55);
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
            break;
        }
        byte upper_data = Serial2.read();
        byte lower_data = Serial2.read();
        byte sum = Serial2.read();
        // Serial.printf("high %u low %u sum %u\r\n", upper_data, lower_data, sum); // for debug
        if (((0xFF + upper_data + lower_data) & 0xFF) == sum) // fix to match JSN-SR04T-2.0 checksum calculation
        {
            uint16_t distance = (upper_data << 8) | (lower_data & 0xFF);
            LOG.printf("Sonic:%u Distance:%umm measurement:%u time left:%u retries left:%u\r\n", barrel, distance, x, timeLeft, retryLeft);
            if (distance)
            {
                if (distance == 10555)
                {
                    measure = x; // number of measurements so far (excluding the last "out of range" measurement)
                    distanceAvearge = 0;
                    if(!ErrorCheck(barrel, BARREL_SONIC_OUTOFRANGE))
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
        {
            if(!ErrorCheck(barrel, BARREL_SONIC_CHECKSUM))
                ErrorSet(barrel, BARREL_SONIC_CHECKSUM);                    
        }
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
        {
            if(!ErrorCheck(barrel, BARREL_SONIC_INACCURATE))
                ErrorSet(barrel, BARREL_SONIC_INACCURATE);
        }
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
    if (!timeLeft)
    {
        // moved over here to prevent MUX Deadlock
        if (!ErrorCheck(barrel, BARREL_SONIC_TIMEOUT))
        {
            ErrorSet(barrel, BARREL_SONIC_TIMEOUT);
            SendSMS("No signal @Ultrasonic", barrel);
        }
    }
} // end SonicMeasure

uint16_t BARRClass::SonicLastMM(byte barrel) { return myBarrel[barrel]._SonicLastValue; }
int16_t BARRClass::SonicLastMMfromEmpty(byte barrel) { return myBarrel[barrel]._SonicOffset - myBarrel[barrel]._SonicLastValue; }
uint16_t BARRClass::SonicOffsetGet(byte barrel) { return myBarrel[barrel]._SonicOffset; }
uint16_t BARRClass::SonicMLinMMGet(byte barrel) { return myBarrel[barrel]._SonicMLinMM; }
void BARRClass::SonicOffsetSet(byte barrel, uint16_t offs) { myBarrel[barrel]._SonicOffset = offs; }
void BARRClass::SonicMLinMMSet(byte barrel, uint16_t coef) { myBarrel[barrel]._SonicMLinMM = coef; }

// sonic calculate liters of last measurement
int16_t BARRClass::SonicCalcLiters(byte barrel)
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
int8_t BARRClass::BarrelPercents(byte barrel)
{
    return 100 * (SonicCalcLiters(barrel) - myBarrel[barrel]._VolumeMin) / (myBarrel[barrel]._VolumeMax - myBarrel[barrel]._VolumeMin);
    // exclude unusable percents below Min point
}

// total liters in all barrels excluding mixing barrel and barrels with errors
// should I remeasure all sonics before?
int16_t BARRClass::SonicLitersTotal()
{
    int16_t result = 0;
    for (byte x = 1; x < NUM_OF_BARRELS; x++)
        if (!ErrorGet(x))                 // if no errors at all
            result += SonicCalcLiters(x); // add this barrel content to sum
    return result;
}

// same as above but exclude unusable liters from below draining point.
// should I remeasure all sonics before?
int16_t BARRClass::SonicLitersUsable()
{
    int16_t result = 0;
    for (byte x = 1; x < NUM_OF_BARRELS; x++)
        if (!ErrorGet(x))                                            // if no errors at all
            result += (SonicCalcLiters(x) - myBarrel[x]._VolumeMin); // add this barrel content minux wasted liters to sum
    return result;
}

// totally empty (by ultrasonic)
bool BARRClass::isDry(byte barrel)
{
    SonicMeasure(barrel);
    return SonicCalcLiters(barrel) < 2; // +1 spare as a safeguard
}

// reached min level (by ultrasonic)
bool BARRClass::isEmpty(byte barrel)
{
    SonicMeasure(barrel);
    return SonicCalcLiters(barrel) <= myBarrel[barrel]._VolumeMin;
}

// reached max level (by ultrasonic)
bool BARRClass::isFull(byte barrel)
{
    SonicMeasure(barrel);
    return SonicCalcLiters(barrel) >= myBarrel[barrel]._VolumeMax;
}
/*-------- Barrels END ----------*/

/*-------- FMSD Begin ----------*/

// open barrels taps and pump (source barrel, target barrel)
void OpenTaps(byte drainBarrel, byte storeBarrel)
{
    // open barrel drain tap
    expanders.DrainingRelay(drainBarrel, true);
    // open barrel store tap
    expanders.StoringRelay(storeBarrel, true);
    // start pump
    expanders.Pump(true);
}

// close barrels taps and pump (source barrel, target barrel)
void CloseTaps(byte drainBarrel, byte storeBarrel)
{
    // stop pump
    expanders.Pump(false);
    // wait untill pressure released
    Alarm.delay(100);
    // close barrel drain tap
    expanders.DrainingRelay(drainBarrel, false);
    // close barrel store tap
    expanders.StoringRelay(storeBarrel, false);
}

void ServiceManual()
{
    if (System.manual_task())
    {
        //LOG.println("test");
        switch (System.manual_task())
        {
            case 1:
            Serial.println("test man case 1");
            //FillManual
            break;
            case 2:
            Serial.println("test man case 2");
            //MixManual
            break;
            case 3:
            Serial.println("test man case 3");
            //StoreManual
            break;
            case 4:
            Serial.println("test man case 4");
            //DrainManual
            break;
        }
        //move to tasks - System.manual_task_reset(); // important - no double-run
        LOG.printf("watermark:%u\r\n", uxTaskGetStackHighWaterMark(loop1));
    }
}

// loops untill stopped state is unset
void StoppedWait()
{
while (System.state_check(STOPPED_STATE))
    {
        ServiceManual();
        Alarm.delay(100); // sleep
    }
}

// closes barrels taps 
// wait until no "stopped state"
// opens taps back
void fmsPause(byte Source, byte Destination)
{
    //SaveStructs(); //disabled for mow....
    LOG.println(F("Status Stopped! auto is paused"));
    System.state_print();
    // if no source barrel = we are filling
    if (Source == 0xFF)
    {
        expanders.FillingRelay(Destination, false);
        StoppedWait();
        expanders.FillingRelay(Destination, true);
    }
    else
    {
        CloseTaps(Source, Destination);
        StoppedWait();
        OpenTaps(Source, Destination);
    }
}

// receives sensor number
// sets stopped state if pressure error
void PressureCheck(byte sens)
{
    pressure.measure(sens);
    switch (pressure.ErrorGet(sens))
    {
        case PRESSURE_NORMAL:
        break;
        case PRESSURE_NOPRESSURE:
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        pressure.measure(sens);
        if (pressure.ErrorGet(sens) == PRESSURE_NOPRESSURE)
            System.state_set(STOPPED_STATE);
        break;
        case PRESSURE_OVERPRESSURE:            
        case PRESSURE_DISCONNECT:            
        case PRESSURE_SHORTCIRCUIT:            
        System.state_set(STOPPED_STATE);
        break;
    }
}

// receives sensor number
// sets stopped state if no flow
void FlowCheck(byte sens)
{
    if (!flow.FlowGet(FLOW_SENSOR_FRESHWATER))
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        if (!flow.FlowGet(FLOW_SENSOR_FRESHWATER))
        {
            System.state_set(STOPPED_STATE);
            expanders.setRGBLED(LED_RED);
            if (sens == FLOW_SENSOR_FRESHWATER)
            {
                if (System.error_check(ERR_WATER_NOFLOW))
                {
                    System.error_set(ERR_WATER_NOFLOW);
                    SendSMS("error: no water flow while Filling. check flowsensor 1, filling solenoid");                
                }
            }
            else
            {
                if (System.error_check(ERR_NUTRI_NOFLOW))
                {
                    System.error_set(ERR_NUTRI_NOFLOW);
                    SendSMS("error: no nutrients flow. check pump, flowsensor 2, solenoids");                
                }
            }
        }
    }
}

void blinkDelay(unsigned long ms, byte color)
{
    Alarm.delay(ms/2);
    expanders.setRGBLED(LED_OFF);
    Alarm.delay(ms/2);
    expanders.setRGBLED(color);
}

// receives barrel to fill, required water level
// checks whatever clean water line have pressure
// fills clean water until level is reached, or barrel is full,
// stops if flowsensor malfunction detected
void Fill(byte barrel, uint16_t requirement)
{
    LOG.printf("filling barrel:%u to %uL\r\n", barrel, requirement);
    flow.CounterReset(FLOW_SENSOR_FRESHWATER); // reset flow counter 1
    expanders.FillingRelay(barrel, true); // open barrel filling tap
    // fill until requirement OR barrel_high_level
    while (!barrels.isFillTargetReached(barrel, BARREL_FRESHWATER, requirement))
    { 
        // LOGIC
        barrels.FreshwaterFillCalc(barrel); // assign flowcount to barrel         
        if (barrels.isFull(barrel)) // if barrel ammount is Full then stop
        {
            LOG.printf("[E] barrel:%u full. breaking..\r\n", barrel);
            break;
        }
        // STOP-CHECK
        if (System.state_check(STOPPED_STATE))
            fmsPause(0xff, barrel);
        // SENSOR CHECK
        if (pressure.Enabled(PRES_SENSOR_FRESHWATER))
            PressureCheck(PRES_SENSOR_FRESHWATER);
        FlowCheck(FLOW_SENSOR_FRESHWATER);
        blinkDelay(1000, LED_YELLOW);
    }
    expanders.FillingRelay(barrel, false); // close tap
    barrels.FreshwaterFillCalc(barrel);  // assign flowcount to barrel
    flow.CounterReset(FLOW_SENSOR_FRESHWATER); // reset flow counter 1
    LOG.printf("END filling barrel:%u to %uL\r\n", barrel, requirement);
}

// will mix "barrel" untill "duration" minutes is over,
// or before if state changed to stopped while we're not operating manual
void Mix(byte barrel)
{
    LOG.printf("mixing barrel:%u for %uMin.\r\n", barrel, System.mix_needed());
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS); // reset flow counter 2
    OpenTaps(barrel, barrel); // open both taps of the same barrel to mix it :)
    // loop - while mix timer > 0
    while (System.mix_needed())
    {
        // LOGIC
        for (byte s=0;s<60;s++)
        {
            // STOP-CHECK
            if (System.state_check(STOPPED_STATE))
                fmsPause(barrel, POOLS);
            // SENSOR CHECK
            if (pressure.Enabled(PRES_SENSOR_NUTRIENTS))
                PressureCheck(PRES_SENSOR_NUTRIENTS);
            blinkDelay(1000, LED_GREEN);
        }
        System.mix_less();// decrement counter every minute
        LOG.printf("barrel:%u %uMin remaining\r\n", barrel, System.mix_needed());
    }
    CloseTaps(barrel, barrel); // counter reached zero
    // report mixed ammount
    LOG.printf("mixed %u liters\r\n", flow.CounterGet(FLOW_SENSOR_NUTRIENTS) / flow.DividerGet(FLOW_SENSOR_NUTRIENTS));
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS); // reset flow counter 2
    System.mix_reset(); // reset mix timer for next run
    LOG.printf("END mixing barrel:%u.\r\n", barrel);
}

// transfers nutrients from "barrel" to "target"
// untill barrel is empty or untill target is full
// or untill "stopped" state set, except if system state also set to manual
void Store(byte barrel, byte target)
{
    LOG.printf("storing from barrel:%u to target %u\r\n", barrel, target);
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS); // reset flow counter 2
    if (barrel == target)
    {
        LOG.println("Error! trying to store to itself.\r\nstoring function exit now.");
        return; // exit right away
    }
    OpenTaps(barrel, target); // drain barrel into target
    // loop - while source barrel not empty AND target not full
    while (!barrels.isEmpty(barrel) && !barrels.isFull(target))
    {
        // LOGIC
        // every 50 liters (not too often for good calculation accuracy)
        if (flow.CounterGet(FLOW_SENSOR_NUTRIENTS) / flow.DividerGet(FLOW_SENSOR_NUTRIENTS) > 50)
            barrels.NutrientsTransferCalc(barrel, target);
        // STOP-CHECK
        if (System.state_check(STOPPED_STATE))
            fmsPause(barrel, POOLS);
        // SENSOR CHECK
        if (pressure.Enabled(PRES_SENSOR_NUTRIENTS))
            PressureCheck(PRES_SENSOR_NUTRIENTS);
        blinkDelay(1000, LED_CYAN);
    }
    CloseTaps(barrel, target);
    barrels.NutrientsTransferCalc(barrel, target); // calculate final ammount
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS); // reset flow counter 2
    LOG.printf("END storing from barrel:%u to target %u\r\n", barrel, target);
}

// draining from barrel, untill requirement liters is transferred, or barrel is empty
// or untill state set to stopped except if state is also manual
void Drain(byte barrel, uint16_t requirement)
{
    uint16_t barrel_before = barrels.NutriGet(barrel);
    LOG.printf("Draining %uL from barrel:%u (%uL)\r\n", System.drain_needed(), barrel, barrel_before);
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS); // reset flow counter 2
    OpenTaps(barrel, POOLS); // open barrel, drain to pools
    // loop while drain counter > 0 and barrel x not empty
    while (System.drain_needed() && !barrels.isEmpty(barrel))
    {
        // LOGIC
        System.drain_substract(barrel);
        // STOP-CHECK
        if (System.state_check(STOPPED_STATE))
            fmsPause(barrel, POOLS);
        // SENSOR CHECK
        if (pressure.Enabled(PRES_SENSOR_NUTRIENTS))
            PressureCheck(PRES_SENSOR_NUTRIENTS);
        blinkDelay(1000, LED_BLUE);
    }
    CloseTaps(barrel, POOLS); // stop pump and taps
    Alarm.delay(1000);
    System.drain_substract(barrel); // last flow calculation
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS); // reset flow counter 2
    LOG.printf("END Draining %uL from barrel:%u\r\n", barrel_before - barrels.NutriGet(barrel), barrel);
}

void fmsTask(void * pvParameters)
{ // Filling Mixing Storing Draining
    LOG.printf(" fmsd begin  system state:%u  watermark:%u\r\n", System.state_get(), uxTaskGetStackHighWaterMark(loop1));
    while (true)
    {
        if (System.state_check(FILLING_STATE))
        {
            LOG.print(F("system running auto - waiting for nutes\r\n\r\n"));
            System.state_set(STOPPED_STATE); // wait untill nutes loaded before filling+mixing
            while (System.state_check(STOPPED_STATE)) // stay here if system stopped
            {
                ServiceManual();
                Alarm.delay(1000);                         // check state every second
            }
            Fill(System.fill_barrel(), System.fill_req_get());
            System.state_set(MIXING_STATE);
            System.state_unset(FILLING_STATE);
            //SaveStructs(); //disabled for mow....
        }

        ServiceManual(); // take the opportunity
        vTaskDelay(1000 / portTICK_PERIOD_MS);   

        if (System.state_check(MIXING_STATE))
        {
            Mix(System.fill_barrel());
            System.state_set(STORING_STATE);
            System.state_unset(FILLING_STATE);
            //SaveStructs(); //disabled for mow....
        }

        ServiceManual(); // take the opportunity
        vTaskDelay(1000 / portTICK_PERIOD_MS);   

        if (System.state_check(STORING_STATE))
        {
            expanders.setRGBLED(LED_CYAN);
            // still have nutes to transfer
            while (!barrels.isEmpty(System.fill_barrel()))
            {
                // first we drain if neccesery
                if (System.drain_needed())
                {
                    System.state_set(DRAINIG_STATE);
                    // drain untill empty or requirement satisfied.
                    Drain(System.fill_barrel(), System.drain_needed());
                    //SaveStructs(); //disabled for mow....
                    if (barrels.isEmpty(System.fill_barrel()))
                        break; // if drained filling barrel to empty - break store loop
                }
                else // no drain requirement
                {
                    System.state_unset(DRAINIG_STATE);                    
                }

                // then we store what is left
                // target not full - transfer into storing barrel
                // excluding the case where all system was full and storing_barrel pointed to barrel 0
                // skip barrel if disabled or errorous!
                byte stor = System.store_barrel();
                if (!barrels.isFull(stor) && !barrels.ErrorGet(stor) && stor > 0)
                {
                    Store(System.fill_barrel(), stor);
                    //SaveStructs(); //disabled for mow....
                }
                // target full - goto next barrel
                else if (stor > 1)
                {
                    System.store_barrel_down();
                }
                else
                { // no more next - all full
                    expanders.setRGBLED(LED_MAGENTA);
                    LOG.println("All barrels full. system stopped. drain to continue");
                    // wait for drain request
                    while (!System.drain_needed())
                    {
                        ServiceManual();
                        Alarm.delay(1000);
                    }
                    // drain the mixer first
                    System.store_barrel_set(System.fill_barrel());
                    // drain untill empty or requirement satisfied.
                    Drain(System.fill_barrel(), System.drain_needed());
                    //SaveStructs(); //disabled for mow....
                }
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            } // got here cause filling_barrel is empty
            // or broke out of the loop cause all including mixer is full --check this again cause system changed!!
            while (System.drain_needed())
            {
                System.state_set(DRAINIG_STATE);
                // storing_barrel not empty? not errorous? drain it
                if (!barrels.isEmpty(System.store_barrel()) && !barrels.ErrorGet(System.store_barrel()))
                {
                    Drain(System.store_barrel(), System.drain_needed());
                    //SaveStructs(); //disabled for mow....
                }
                // storing_barrel empty but not the last barrel (i filled from last to first)  // try next barrel
                else if (System.store_barrel() < NUM_OF_BARRELS - 1)
                    System.store_barrel_up();
                // all storage barrels empty?
                else
                    break; // totally empty - will do another cycle fms to refill
            }
            if (!System.drain_needed())
                System.state_unset(DRAINIG_STATE);
            // repeat the fms cycle if no draining required, or all barrels empty
            System.state_set(FILLING_STATE);
            System.state_unset(STORING_STATE);

            //SaveStructs(); //disabled for mow....
        } // if storing_state
    }     // endless loop ends here :)
}
/*-------- FMSD END ----------*/

/*-------- WebServer Begin ----------*/
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

    server.on("/index", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
            request->send(*disk, "/index.html", String(), false);
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
        response->print("<button onclick=\"location=\'/manual\'\">manual controls</button><span> </span>");
        response->print("<button onclick=\"location=\'/fmsd\'\">manual fmsd</button><br>");
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

        response->print("</ul>");
        response->print("<button onclick=\"location=location\">reload</button><span> </span>");
        response->print("<button onclick=\"location=\'/reset\'\">reset</button><span> </span>");
        response->print("<button onclick=\"location=\'/list\'\">list filesystem</button><span> </span>");
        response->print("<button onclick=\"location=\'/fmsd\'\">manual fmsd</button><br>");
        response->printf("<span>uptime: %lli seconds. system state:%u</span><br><br>", esp_timer_get_time() / 1000000, System.state_get());
        response->print("</body></html>");
        request->send(response);
    });

    server.on("/fmsd", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n\r\n", request->url().c_str());
        byte status = 0; // for parameter out of range error
        if (request->hasArg("task"))
        {
            // converting arguments
            int task = request->arg("task").toInt();
            int ammo = request->arg("ammo").toInt();
            int src = request->arg("src").toInt();
            int dest = request->arg("dest").toInt();
            // printing message
            LOG.printf("[man]fmsd task received: task%i ammo%i src%i dest%i\r\n", task, ammo, src, dest);
            // checking input is valid
            if (src > -1 && src < NUM_OF_BARRELS && task > -1 && task < 5 && ammo > -1 && ammo < 32768 && dest > -1 && dest < NUM_OF_BARRELS)
            {
                // running the task
                System.manual_task_set(task, src, dest, ammo);
                status = 1;
            }
            else
            {
                LOG.println("[E] parameter out of range");
                status = 2;
            }
        }
        AsyncResponseStream *response = request->beginResponseStream("text/html");
        response->print("<html><body style=\"transform: scale(2);transform-origin: 0 0;\"><h3>manual F.M.S.D.</h3><ul>");
        response->print("<form action=\"/fmsd\">");
        response->print("<li>fill-1 mix-2 store-3 drain-4</li>");
        response->printf("<li><input id=\"task\" name=\"task\" value=\"%u\"></li>", System.manual_task());
        response->print("<li>how much</li>");
        response->printf("<li><input id=\"ammo\" name=\"ammo\" value=\"%u\"></li>", System.manual_howmuch());
        response->print("<li>from</li>");
        response->printf("<li><input id=\"src\" name=\"src\" value=\"%u\"></li>", System.manual_from());
        response->print("<li>to</li>");
        response->printf("<li><input id=\"dest\" name=\"dest\" value=\"%u\"></li>", System.manual_to());
        switch (status)
        {
            case 0:
            response->print("<li>insert data and press Go</li>");
            break;
            case 1:
            response->print("<li>ok, data received</li>");
            break;
            case 2:
            response->print("<li>out of range data, try again!</li>");
            break;
        }
        response->print("<li><input type=\"submit\" value=\"Go\"><span> </span><button type=\"reset\" onclick=\"location=\'/fmsd?task=0\'\">cancel fmsd</button>");
        response->print("</form>");
        response->print("</ul>");
        response->print("<button onclick=\"location=\'/list\'\">list filesystem</button><span> </span>");
        response->print("<button onclick=\"location=\'/manual\'\">manual controls</button><br>");
        response->printf("<span>uptime: %lli seconds. system state:%u</span><br><br>", esp_timer_get_time() / 1000000, System.state_get());
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
        }
        else
            request->send(200, "text/plain", "no args!");
        request->redirect("/manual");
    });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        request->redirect("/manual");
        vTaskDelay(100); // to prevent reset before redirect
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
    LOG.print(F("-Server init\r\n"));
}
/*-------- WebServer END ----------*/

/*-------- NTP Begin ----------*/
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
/*-------- NTP END ----------*/

void LoadStructs()
{
    if (isSD)
        Restore(); // in case something is missing in the SD
    LOG.printf("Loading system from %s\r\n", isSD ? "SD" : "SPIFFS");
    if (disk->exists("/SysState.bin"))
        System.LoadSD();
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
        System.SaveSD();

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

    //initiate expander ports - I2C wire
    expanders.Init();

    // initiate SD Card - SPI bus + SPIFFS
    initStorage();

    // load structs from SD card
    LoadStructs();

    pressure.setSensor(PRES_SENSOR_FRESHWATER, PRESSUR_1_PIN);
    pressure.setSensor(PRES_SENSOR_NUTRIENTS, PRESSUR_2_PIN);

    // attach interrupts for flow sensors
    flow.begin();
    flow.CounterReset(FLOW_SENSOR_FRESHWATER);
    flow.CounterReset(FLOW_SENSOR_NUTRIENTS);

    // attach interrupts for start-stop buttons 
    System.begin();

    // initialize uart2 SIM800L modem at MUX port 7?
    modemInit();

    // Create tasks
    xTaskCreatePinnedToCore(fmsTask, "loop1", 10000, (void *)1, 1, &loop1, ARDUINO_RUNNING_CORE);

    //WiFiManager Local intialization. Once its business is done, there is no need to keep it around
    AsyncWiFiManager wifiManager(&server, &dns);
    //wifiManager.resetSettings();
    wifiManager.setDebugOutput(false);
    wifiManager.setConfigPortalTimeout(180);  // 3 minutes
    wifiManager.autoConnect("AutoConnectAP"); // will stop here if no wifi connected
    LOG.printf("ESSID: %s IP: %s\r\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

    // setup NTP

    //start Web Server
    setupServer();
    ArduinoOTA.setHostname("barrels");
    ArduinoOTA.begin();
    //mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    MDNS.addService("http", "tcp", 80); // add mDNS http port

   expanders.setRGBLED(LED_OFF);
    //SendSMS("System Started");
}

void loop()
{
    ArduinoOTA.handle();
    System.state_change_check();
    vTaskDelay(1);
    //vTaskDelay(portMAX_DELAY); // wait as much as posible ...
}