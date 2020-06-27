/*
bugs:
*Guru Meditation Error: Core  1 panic'ed (LoadProhibited)
while restoring from spiffs to sd without sd loaded
*

to implement:
ultrasonic multiple measurements + timeout
barrels - continue
ultrasonic - add to /manual
make pressure sensor interrupt? or check constantly so i can turn off the pump on overpressure - task loop inside other cpu core?
apply test code to check every part of the system:
    ultrasonic data - after barrels done - at manual page
    start-stop via serial.print
    ntp sync
make filling mixing storing draining into single Task?
reimplement global save function:
    wait STOPPED_STATE, wait untill flow stopped, save 
pressure sensors - "stop pump on overpressure" Task
deprecate RTC? replace with ntp + timeAlarms?
add everything to setup 
start/stop interrupts
    start sets canMix global boolean
        task or loop reads the bool - changes RGB immediately, but start mixing task only when no tasks running
    stop sets shouldStop global boolean
        task or loop reads the bool - changes RGB immediately, triggers stop and save
    on stop:     pressure.SaveSD(); flow.SaveSD();
    // can set the bool to !bool to check if pressed twice - for reset and other system tasks
rgb led status (read from system status via error-reporting task?)
work on system hardware 
sendSMS should return actual error names - dictionary from file?
reimplement sendSMS as a buffer, actual send when isSmsToSend true and mux not busy
send sms on SD card error!

add to schematic:
sim800 4v 2a power line
rj45 colors to pin numbers
LED, start/stop pin numbers rj45 socket
rtc module
12v to 5v to 3.3v power line



optional:
switchFS add fs check, try to restart filesystem on failure
add ... on serial.available to check module awake
sendSMS - rewrite String to Char Array
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
#include <ESPAsyncWiFiManager.h>         //https://github.com/tzapu/WiFiManager

//#include <Hash.h>
#include <AsyncTCP.h>
#include <ArduinoOTA.h>
//https://github.com/jandrassy/TelnetStream
#include "TimeAlarms.h" 

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif


//hex codes for "error_state" bit-field
#define NOWATER_ERROR	    0x01    //1-1-water line no pressure
#define PS1TOOHIGH_ERROR    0x08    //4-8-pump1 pressure too high - storing solenoids malfunction
#define PS1TOOLOW_ERROR     0x10    //5-16-pump1 pressure too low - pump failure or no more liquid
#define PS2TOOHIGH_ERROR    0x20    //6-8-pump1 pressure too high - storing solenoids malfunction
#define PS2TOOLOW_ERROR     0x40    //7-16-pump1 pressure too low - pump failure or no more liquid
#define FS1_NOFLOW_ERROR    0x80    //8-128-fs1 no flow
#define FS2_NOFLOW_ERROR    0x100   //9-256-fs2 no flow
#define NOMAINS_ERROR       0x400   //B-1024-no mains power
#define BARRELS_ERROR       0x800   //C-2048-barrels error

//hex codes for "state" bit field
#define MANUAL_STATE    0x20    // manual mode on
#define FILLING_STATE   0X10    // F = filling task on
#define MIXING_STATE    0x08    // M = mixing task on
#define STORING_STATE   0x04    // S = storing task on
#define DRAINIG_STATE   0x02    // E = draining task on
#define STOPPED_STATE   0x01    // X = stopped status on

// RGB_LED bitmap 00000BGR
#define LED_OFF     0x00 //000
#define LED_RED     0x01 //001
#define LED_GREEN   0x02 //010
#define LED_YELLOW  0X03 //011
#define LED_BLUE    0x04 //100
#define LED_MAGENTA 0X05 //101
#define LED_CYAN    0X06 //110
#define LED_WHITE   0X07 //111

#define MUX_UNLOCKED 255 //valid shannels are 0-15

#define OUT_PORT Serial //SerialAndTelnet or file
#define SYNC_INTERVAL 600 // NTP sync - in seconds
#define NTP_PACKET_SIZE 48
// NTP time is in the first 48 bytes of message
#define NTP_HOSTNAME "pool.ntp.org"
#define NTP_PORT 123
#define NTP_TIMEOUT 5000
#define LOCAL_UDP_PORT 8888  // local port to listen for UDP packets

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
FS* disk = &SPIFFS; // default
File file;
WiFiUDP UDP;
AsyncWebServer server(80);
DNSServer dns;
bool isTimeSync = false;
bool isSaving = false;
bool isSD = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // for interrupts and xTasks

//function declarations:
bool Load(const char* fname, byte* stru_p, uint16_t len);
bool Save(const char* fname, byte* stru_p, uint16_t len);
void SaveStructs();
void IRAM_ATTR FlowSensor1Interrupt();
void IRAM_ATTR FlowSensor2Interrupt();


/*-------- Filesystem code ----------*/
// SD Card and SPIFFS
void initStorage(){
    byte retry=5; // 5 times for SD
    while (retry) {
        if(SD.begin()){
            OUT_PORT.println(F("SD card OK"));
            disk=&SD;
            isSD=true;
            break;
        }
        else{
            OUT_PORT.println(F("Error: Failed to initialize SD card"));
            Alarm.delay(1000);
            retry--; // prevent dead loop 
        }
    }
    retry=3; // 3 times for SPIFFS
    while (retry) {
        if(SPIFFS.begin()){
            OUT_PORT.println(F("SPIFFS OK"));
            break;
        }
        else{
            OUT_PORT.println(F("Error: Failed to initialize SPIFFS"));
            Alarm.delay(1000);
            retry--; // prevent dead loop 
        }
    }
}



// loading Structs from files
// filename, struct pointer, struct lenght "sizeof(myStruct)"
bool Load(const char* fname, byte* stru_p, uint16_t len){
    uint16_t count=0;
    OUT_PORT.printf("Loading %s\t", fname);
    file = disk->open(fname, "r"); 
        if (!file)
        OUT_PORT.print(F("unable to open file\r\n\r\n"));
        else
        for (; count<len; count++) 
            if ( file.available())
            *( stru_p + count ) = file.read();
    #ifdef DEBUG
    OUT_PORT.printf("%s\r\n%u out of %u Bytes read. filesize %satch.\r\n", count==len?"successfully":"failed", count, len, len==file.size()?"M":"Mism");
    #else
    OUT_PORT.println();
    #endif
    file.close();
    return count == len;
}


bool Save(const char* fname, byte* stru_p, uint16_t len){
    uint16_t count=0;
    OUT_PORT.printf("Saving %s\t", fname);
    file = disk->open(fname, "w"); // creates the file of not exist
    //file.setTimeCallback(timeCallback);
    if (!file)
        OUT_PORT.print(F("unable to open file\r\n\r\n"));
    else
        count = file.write(stru_p, len);
    #ifdef DEBUG
    OUT_PORT.printf("%s\r\n%u out of %u Bytes writen. filesize %satch.\r\n", count==len?"successfully":"failed", count, len, len==file.size()?"M":"Mism");
    #else
    OUT_PORT.println();
    #endif
    file.close();
    return count == len;
}


// backup overwrites all files on SPIFFS
byte Backup(){
    Serial.println(F("Backing up all files from SD to SPIFFS"));
    File dir = SD.open("/");
    File file = dir.openNextFile();
    size_t len=0; // file chunk lenght at the buffer
    byte counter=0;
    while(file){
        static uint8_t buf[512];
        if(!file.size()){ Serial.printf("skipping empty file %s\r\n", file.name()); }
        else {
            // was unable to detect SD disconnection - file was true, name and size returned valid, only read returned zero.
            if(!file.read( buf, 512)) {Serial.println(F("[E][:231] SD not exist!!"));break;} // protect against endless loop on SD error
            file.seek(0); // start from start
            File destFile = SPIFFS.open(file.name(), FILE_WRITE);
            while( file.available() ){
                len = file.read( buf, 512);
                //stream.readBytes(buffer, length)
                destFile.write( buf, len );
                Serial.printf("copying %s from SD to SPIFFS %u bytes copied\r\n", destFile.name(), len);
            }
            destFile.close();          
            counter++;  
        }
        file.close();
        file = dir.openNextFile();
    }
    dir.close();
    Serial.printf("Backup finished. %u files copied\r\n", counter);
    return counter;
}

// restore only files missing on SD card
byte Restore(){
    Serial.println(F("Restoring missing files from SPIFFS to SD"));
    File dir = SPIFFS.open("/");
    File file = dir.openNextFile();
    size_t len=0; // file chunk lenght at the buffer
    byte counter=0;
    while(file){
        //file.name() file.size());
        if(SD.exists(file.name()))
            Serial.printf("file %s already exist on SD\r\n", file.name());
        else if (!file.size())
            Serial.printf("skipping empty file %s\r\n", file.name());
        else {
            File destFile = SD.open(file.name(), FILE_WRITE);
            Serial.println(file.name());
            if(destFile){
                static uint8_t buf[512];
                //memset(buf, 0, 512); // zerofill the buffer
                while( file.available() ){
                    len = file.read( buf, 512);
                    destFile.write( buf, len );
                    Serial.printf("copying %s from SPIFFS to SD %u bytes copied\r\n", destFile.name(), len);
                }                
            }
            else {
                Serial.println(F("Error writing to SD"));
            }
            destFile.close();
            if (file.size() != destFile.size())
                Serial.printf("[E] file %s size mismatch!\r\n", file.name());
            else
                counter++;
        }
        file.close();
        file = dir.openNextFile();
    }
    dir.close();
    Serial.printf("Restore finished. %u files copied\r\n", counter);
    return counter;
}
/*-------- Filesystem END ----------*/








// declare I/O Expander ports and constants

// MCP23017 with pin settings
MCP23017 expander1(0); // Base Address + 0: 0x20
MCP23017 expander2(1); // Base Address + 1: 0x21

class exp {
private:
    uint8_t _muxLock = MUX_UNLOCKED; // unlocked

public:
    void Init(){
        // initialize TwoWire communication
        Wire.begin();

        for (size_t i = 0; i < 16; i++) {
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
    void setMUX(uint8_t address){

        if ((_muxLock != address) && (_muxLock != MUX_UNLOCKED))
            OUT_PORT.printf("-MUX! previously locked to %u, %u is waiting\r\n", _muxLock, address);
        while ((_muxLock != address) && (_muxLock != MUX_UNLOCKED))
            Alarm.delay(1);

        OUT_PORT.printf("-MUX! is free. locking to %u\r\n", address);
        _muxLock = address;

        for (uint8_t i = 0; i < 4; i++){
            //checks bit 0-3 of mux Address "address"
            //if bit set - setValue receives a positive value
            //otherwise sets setValue with 0
            //offset of 0 - expander 0x20 pins a0 a1 a2 a3
            expander1.getPin(i+0).setValue( address & (1 << i) ); 
        }
        expander1.write();
    }

    uint8_t GetMUX(){
        return _muxLock;
    }

 
    // very important to run this every time you ended up business with setMUX
    void UnlockMUX(){
        if (_muxLock != MUX_UNLOCKED){
        OUT_PORT.printf("-MUX! Unlocking %u\r\n", _muxLock);
        _muxLock = MUX_UNLOCKED;
        }
        else
        OUT_PORT.println(F("-MUX! already unlocked!"));
    }

    void setRGBLED(uint8_t address){
        for (uint8_t i = 0; i < 3; i++){
            //checks bit 0-2 of color ( R G B ) in "address"
            //offset of 4 - expander 0x20 pins a4 a5 a6
            expander1.getPin(i+4).setValue( address & (1 << i) ); 
        }
        expander1.write();
    }

    void Protect(bool state){
        // triggers last relay in each relay board to disconnect 12v line
        expander1.getPin(15).setValue( !state ); // filling relay protect pin
        expander2.getPin( 7).setValue( !state ); // storing relay protect pin
        expander2.getPin(15).setValue( !state ); // draining relay protect pin
        expander1.write();
        expander2.write();
    }

    void PumpIn(bool state){
        // triggers dIN solenoid and then the pump
        expander2.getPin( 6).setValue( !state ); // storing relay dIN pin
        expander2.write();
        expander1.getPin(14).setValue( !state ); // filling relay pump pin
        expander1.write();
    }

    void PumpOut(bool state){
        // triggers dOUT solenoid and then the pump
        expander2.getPin(14).setValue( !state ); // draining relay dOUT pin
        expander2.write();
        expander1.getPin(14).setValue( !state ); // filling relay pump pin
        expander1.write();
    }

    void FillingRelay(uint8_t address, bool state){
        //offset of 8 - expander 0x20 pins b0-b7
        expander1.getPin( address+8 ).setValue( !state ); 
        expander1.write();
    }

    void StoringRelay(uint8_t address, bool state){
        //offset of 0 - expander 0x21 pins a0-a7
        expander2.getPin( address ).setValue( !state ); 
        expander2.write();
    }

    void DrainingRelay(uint8_t address, bool state){
        //offset of 8 - expander 0x21 pins b0-b7
        expander2.getPin( address+8 ).setValue( !state ); 
        expander2.write();
    }

} expanders; // initiated globally






// set MUX to SIM_MUX_ADDRESS
// send sms with error code description
void SendSMS(String message){
    OUT_PORT.print("-sendsms: ");
    OUT_PORT.println(message);
    expanders.setMUX(7); // modem is at port 7
    Alarm.delay(10); // wait until expander + mux did their job
    Serial2.println("AT+CMGS=\"+972524373724\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
    Serial2.print( message ); //text content
    Serial2.write(26); // send ctrl+z end of message
    //Serial2.print((char)26); // if the above won't work - thy this one instead
    expanders.UnlockMUX();
}
// module needs 3.4V to 4.4V (Ideal 4.1V) @ 2amp

/*
  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  mySerial.println("AT+CREG?"); //Check whether it has registered in the network
  ATI – Get the module name and revision
AT+COPS? – Check that you’re connected to the network, in this case BSNL
AT+COPS=? – Return the list of operators present in the network.
AT+CBC – will return the lipo battery state. The second number is the % full (in this case its 93%) and the third number is the actual voltage in mV (in this case, 3.877 V)
  */

void modemInit(){
    OUT_PORT.println("-modem init");
    expanders.setMUX(7); // modem is at port 7
    Alarm.delay(10); // wait until expander + mux did their job
    // Serial2.begin(9600, SERIAL_8N1); // already done in main
    Serial2.println("AT"); //Once the handshake test is successful, it will back to OK
    Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
    expanders.UnlockMUX(); // Must unlock after every use!!
}

struct myST { 
    //bit field
    //00NFMSEX
    // N = maNual mode on - overrides stopped
    // F = Filling task on
    // M = Mixing task on
    // S = Storing task on
    // E = Emptying task on - overrides f,m,s
    // X = stopped status on
    uint8_t _state_now = 0;
    uint8_t _state_before = 0;

    // bit field
    //000CBA987654321
    //0-0-no errors
    //1-1-water line no pressure
    //2-2-
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

class STClass{
private:
    myST myState;

public:
    bool LoadSD(){ return Load("/SysState.bin", (byte*)&myState, sizeof(myState)); }

    bool SaveSD(){ return Save("/SysState.bin", (byte*)&myState, sizeof(myState)); }

    uint8_t state_get(){ return myState._state_now; }

    void state_set(uint8_t mask){ myState._state_now |= mask; }

    void state_unset(uint16_t mask){ myState._state_now &= ~mask; }

    void state_save(){ myState._state_before = myState._state_now; } // preserve prevoius state

    void state_load(){ myState._state_now = myState._state_before; } // restore previous state

    // returns true if state have "mask-bit" state on. ex: return_state(MIXING_STATE);
    bool state_check(uint8_t mask){ return myState._state_now & mask; }

    // returns error state
    uint16_t error_get(){ return myState._error_now; }

    void error_set(uint16_t error){ myState._error_now |=  error; }

    void error_unset(uint16_t error){ myState._error_now &= ~error; }

    //returns the difference between current and last error states
    // !! need to reimplement to return either the new error position
    // so I dont have to use uint16_t for return
    // or either if two errors should be reported at once - a while loop that solves them one by one
    uint16_t error_getnew(){
        // bitwise - diff between "before" and "now", 
        // compare with "now" to extract new bits only
        return (myState._error_now ^ myState._error_before) & myState._error_now ; 
    }

    void error_reported(){ myState._error_before=myState._error_now; }

} SystemState;

String system_error(uint16_t error){
    return "system error " + String(error);     // need to reimplement to send error description instead of error number
}

// Task 0 - Error reporting task
void errorReportTask(){
    // while there is no new errors
    while (!SystemState.error_getnew()){
        Alarm.delay(REPORT_DELAY);     //wait - endless loop untill error status changes
    }

    // goes here if there is an error to report
    uint16_t newerrors = SystemState.error_getnew();
    uint16_t counter = 1;
    while(newerrors){ // loops untill error is empty
        if(newerrors & counter){ // try each error state bit one by one
            Serial.printf("system error %u", counter);
            //SendSMS( system_error(counter) );
            newerrors-=counter; // substract what already reported
            counter *=2; // next bit
        }
    }
    // clear error after all being reported
    SystemState.error_reported();
}























































    // flow Sensors Pin Declarations
    // and Interrupt routines


struct myFS{
    volatile uint32_t counter = 0;
    uint16_t conversion_multiplier = 450;
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

class FSClass {
private:
    myFS fsensor[2]; // two sensors
public:
    uint16_t FlowGet(uint8_t sens){
        if ((millis()-fsensor[sens].lastMilis)>1000){ // measurement older than 1 second means no flow
            return 0;}
        return ( (1000000 / (fsensor[sens].flow?fsensor[sens].flow:1) ) / fsensor[sens].conversion_multiplier ); // in mililiters per second
    } // Guru Meditation Error: Core  1 panic'ed (IntegerDivideByZero). Exception was unhandled.

    void IRAM_ATTR CounterInc(uint8_t sens){
        fsensor[sens].counter++;
        fsensor[sens].flow = millis()-fsensor[sens].lastMilis ; // interval in miliseconds from the last call
        fsensor[sens].lastMilis = millis();
    }

    uint64_t CounterGet(uint8_t sens){ return fsensor[sens].counter; }
    
    void CounterSubstract(byte sens, uint64_t value) { fsensor[sens].counter-=value;}
    
    void CounterReset(uint_fast8_t sens){ fsensor[sens].counter=0; }
    
    uint16_t MultGet(uint8_t sens){ return fsensor[sens].conversion_multiplier; }

    void MultSet(uint8_t sens, uint16_t mult){ fsensor[sens].conversion_multiplier = mult; }

    bool LoadSD(){ return Load("/Flow.bin", (byte*)&fsensor, sizeof(fsensor)); }

    bool SaveSD(){ return Save("/Flow.bin", (byte*)&fsensor, sizeof(fsensor)); }

    void begin(){ // attach interrupts
        Serial.printf("-Flow: init sensors at pins %u, %u\r\n", FLOW_1_PIN, FLOW_2_PIN);
        pinMode(FLOW_1_PIN, INPUT_PULLUP);
        pinMode(FLOW_2_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(FLOW_1_PIN), FlowSensor1Interrupt, FALLING);
        attachInterrupt(digitalPinToInterrupt(FLOW_2_PIN), FlowSensor2Interrupt, FALLING);
    }

} flow;

//flow sensor 1 interrupt routine
void IRAM_ATTR FlowSensor1Interrupt() {
    portENTER_CRITICAL_ISR(&mux);
    flow.CounterInc(0);
    portEXIT_CRITICAL_ISR(&mux);
}
//flow sensor 2 interrupt routine
void IRAM_ATTR FlowSensor2Interrupt() {
    portENTER_CRITICAL_ISR(&mux);
    flow.CounterInc(1);
    portEXIT_CRITICAL_ISR(&mux);
}



















// Pressure Sensors Pin Declarations
// and Analog reads
// task pressure sensors - stops pumps on overpressure

struct myPS {
    uint8_t _sensorPin = 255; // defaults
    uint8_t _multiplier = 1;
    uint8_t _offset = 0;
    uint8_t _max_pressure = 255;
    uint8_t _min_pressure = 0;
    uint16_t _TooLowErr = 0;
    uint16_t _TooHighErr = 0;
};

// pressure sensors starts from 0
class pressure_sensor {
private:
    myPS psensor[2]; // two sensor
public:
    // init the sensor
    // !! need to reimplement to use error mask position instead the whole uint16_t mask
    void setSensor(uint8_t num, uint8_t sensorPin, uint16_t TooLowErr, uint16_t TooHighErr){
        psensor[num]._sensorPin = sensorPin;
        psensor[num]._TooLowErr = TooLowErr;
        psensor[num]._TooHighErr = TooHighErr;
        pinMode(sensorPin,INPUT); // initialize analog pin for the sensor
    }

    uint8_t MultiplierGet(uint8_t num) { return psensor[num]._multiplier; }
    void MultiplierSet(uint8_t num, uint8_t mult){ psensor[num]._multiplier = mult; }
    uint8_t OffsetGet(uint8_t num) { return psensor[num]._offset; }
    void OffsetSet(uint8_t num, uint8_t offs){ psensor[num]._offset = offs; }
    uint8_t MaxGet(uint8_t num) { return psensor[num]._max_pressure; }
    void MaxSet(uint8_t num, uint8_t max){ psensor[num]._max_pressure = max; }
    uint8_t MinGet(uint8_t num) { return psensor[num]._min_pressure; }
    void MinSet(uint8_t num, uint8_t min){ psensor[num]._min_pressure = min; }

    bool LoadSD(){ return Load("/Pressure.bin", (byte*)&psensor, sizeof(psensor)); }

    bool SaveSD(){ return Save("/Pressure.bin", (byte*)&psensor, sizeof(psensor)); }

    // read sensor - convert analog value to psi
uint16_t measure(uint8_t sens){
    // convert analog value to psi
    uint16_t pressure = (analogRead(psensor[sens]._sensorPin) * psensor[sens]._multiplier - psensor[sens]._offset) /1000; //test - needs to be replaced with acrual formula
    if (pressure < psensor[sens]._min_pressure) {
        // set underpressure error
        SystemState.error_set(psensor[sens]._TooLowErr);
    }
    if (pressure > psensor[sens]._max_pressure) {
        // set overpressure error
        SystemState.error_set(psensor[sens]._TooHighErr);
        // !! implement - stop the pump !!!
    }

    return pressure;
}

} pressure;


















struct myBR {
    //bit field
    //76543210
    //0 - flush solenoid error
    //1 - store solenoid error
    //2 - drain solenoid error
    //3 - Ultrasonic sensor checksum error
    //4 - Ultrasonic sensor  timeout error
    //5 - 
    //6 - disabled manually
    //7 - other error
    uint8_t _ErrorState=0;
    uint8_t _BarrelNumber=0;       // important for mux selection // really needed?
    uint16_t _VolumeFreshwater=0;  // data from flow sensor
    uint16_t _VolumeNutrients=0;   // data from flow sensor
    uint16_t _VolumeEmpty=0;       // set once at calibration - for sonic sensors
    uint16_t _VolumeMin=0;         // set once at calibration - for flow and sonic sensors
    uint16_t _VolumeMax=1;         // set once at calibration - for flow and sonic sensors
    uint8_t _SonicCoefficient=1;   // set once at calibration - cm to Liters - for sonic sensor
    uint8_t _SonicOffset=0;        // set once at calibration - for sonic sensor
    byte _SonicLastValue=0;
};


class BARRClass{
private:
    myBR myBarrel[8];

public:
    bool LoadSD(){ return Load("/Barrels.bin", (byte*)&myBarrel, sizeof(myBarrel)); }

    bool SaveSD(){ return Save("/Barrels.bin", (byte*)&myBarrel, sizeof(myBarrel)); }

    // error get error set
    byte ErrorGet(byte barrel){ return myBarrel[barrel]._ErrorState; }
    void ErrorSet(byte barrel, byte mask){ myBarrel[barrel]._ErrorState |= mask; }
    void ErrorUnset(byte barrel, byte mask){ myBarrel[barrel]._ErrorState &= ~mask; }
    void ErrorReset(byte barrel){ myBarrel[barrel]._ErrorState=0; }

    uint16_t FreshGet(byte barrel){ return myBarrel[barrel]._VolumeFreshwater; }
    uint16_t NutriGet(byte barrel){ return myBarrel[barrel]._VolumeNutrients; }

     // add fresh flow couter to barrel, then substract it from flowsensor   
    void FreshFlowSetTo(byte barrel){
        uint64_t CounterNow = flow.CounterGet(0); // may be increased during calculation cause flowsensor works on interrupts
        myBarrel[barrel]._VolumeFreshwater+=CounterNow;
        flow.CounterSubstract(0, CounterNow); // counter 0 is freshwater
    }

    // add nutri flow couter to barrel, then substract it from flowsensor
    void NutriFlowSetTo(byte barrel){
        uint64_t CounterNow = flow.CounterGet(1);
        myBarrel[barrel]._VolumeFreshwater+=CounterNow;
        flow.CounterSubstract(1, CounterNow); // counter 1 is nutrients
    }

    // add - NutriFlowTransferTo

//barrels calculations:
//(barrelsEmptyHeight - ultrasonicValue) * barrelsCoefficent = value in liters
//(barrelsEmptyHeight - barrelsFullHeight) / ultrasonicValue = barrels percent 

//    return ( get_flow_volume() + get_sonic_volume() ) / 2; // fifty-fifty - need to change?

 // 0 to 100% from flow 

  // 0 to 100% from ultrasonic 

 // 0 to 100% from ultrasonic and flow combined


//  if (barrelNumber < 0 || barrelNumber >= NUM_OF_BARRELS)
//    barrelNumber = 0; // playing safe with dummys :-)


    uint16_t VolumeMax(byte barrel){
        return myBarrel[barrel]._VolumeMax;
    }

    uint16_t VolumeMin(byte barrel){
        return myBarrel[barrel]._VolumeMin;
    }


    // !! reimplement remeasuring 3 times and return avearge
    // retry on checksum error - count checksum errors
    // retry on timeout - count timeout errors
    // if errors > 10 - set errorstate accordingly

    // returns distance in mm
    uint16_t Sonic(byte barrel){ // the hedgehog :P
        expanders.setMUX(barrel);
        Serial2.write(0x01); // send data so sonic will reply
        while(!Serial2.available()){}; // wait untill data received // !! reimplement with timeout !!!
        if (Serial2.read() == 0xFF ) { //start
            while(Serial2.available()<3){};//wait for all data to be buffered
            uint8_t upper_data = Serial2.read();
            uint8_t lower_data = Serial2.read();
            uint8_t sum = Serial2.read();
            if (((upper_data + lower_data) & 0xff) == sum) {
                uint16_t distance = (upper_data << 8) | (lower_data & 0xff);
                Serial.printf("Sonic %u Distance %u mm\r\n", barrel, distance);
                expanders.UnlockMUX(); // important!
                return distance;
            }
            else {
                Serial.println("checksum error");
            }
            
        }
        expanders.UnlockMUX(); // important!
        return 0;
    }

    // sonic measure
    uint16_t SonicMeasure(byte barrel){
        // sonic() - _SonicOffset = full barrel point 0
        // volumeFull - ("point zero" * _SonicCoefficient) = current barrel volume in liters from sonic
        return myBarrel[barrel]._VolumeMax - ((Sonic(barrel) - myBarrel[barrel]._SonicOffset) * myBarrel[barrel]._SonicCoefficient); // untill impl
    }

    // totally empty
    bool isDry(byte barrel){
        return SonicMeasure(barrel) <= myBarrel[barrel]._VolumeEmpty;
    }

    // reached min level
    bool isEmpty(byte barrel){
        return SonicMeasure(barrel) <= myBarrel[barrel]._VolumeMin;
    }

    // reached max level
    bool isFull(byte barrel){
        return SonicMeasure(barrel) >= myBarrel[barrel]._VolumeMax;
    }

    // concentration in persents
    // 100 * myBarrel[8]._VolumeNutrients / myBarrel[8]._VolumeFreshwater

    void Init(){ // really needed?
        for (byte x=0;x<8;x++)
            myBarrel[x]._BarrelNumber=x; // sets each barrel's number
    }

} barrels;




struct st {
        uint16_t prefill_requirement = 100; // initial 100 liters
        uint16_t prefill_counter = 0;
        uint16_t afterfill_requirement = 400; // additional 400 liters
        uint16_t afterfill_counter = 0;
        uint16_t mix_requirement = 30; // minutes to mix
        uint16_t mix_counter = 0;
        // transfer requirement calculated dynamically by barrel ammount
        // transfer counter applied dynamically at transfer from source to destination
        uint16_t drain_requirement = 0; //in liters - how much to drain
        uint16_t drain_counter = 0;    //in liters - how much left to drain
        uint8_t filling_barrel = 0; // mixer barrel
        uint8_t mixin_barrel = 0;
        uint8_t storing_barrel = 1; // first barrel
        uint8_t draining_barrel = NUM_OF_BARRELS-1; // last barrel (-1 cause we start from zero)
} Transfers;

void Fill(uint16_t barrel, uint16_t requirement){
        // if barrel ammount less than requirement
        // open barrel filling tap
        // fill until requirement OR barrel_high_level
            // decrement requirement by flow counter
            // break if stopped but not manual
        // close mixer tap
}

// Filling task
void FillingTask(){
    while (true){ // endless loop Filling task
        // stopped status off?
        // check water level is prefill_requirement
            // filling(filling_barrel, prefill_requirement)
        // set stopped status on
        // set mixing status on
        // wait untill not stopped status
        // if water level is afterfill_requirement
            // filling(filling_barrel, afterfill_requirement)
        // set filling status off
        // init save
    } // endless loop Filling task
}

void Mix(uint16_t barrel, uint16_t requirement){
    // open barrel drain tap
    // open barrel store tap
    // open dIN tap
    // start pump
    // loop - while requirement > 0
        // decrement requirement every minute
        // break if stopped but not manual
    // if requirement reached 0
        // stop pump
        // close barrel drain tap
        // close dIN tap
        // close barrel store tap
        // init save 
}

// Mixing task
void MixingTask(){
    while(true){ // endless loop Mixing task
        // stopped status off?
            // Mixing(mixin_barrel, mix_requirement)
        // Storing status on
        // mixing status off
        // init save 
    } // endless loop Mixing task
}


void Store(uint16_t barrel, uint16_t target, uint16_t requirement){
    // open barrel drain tap
    // open target store tap
    // open dIN tap
    // start pump
    // loop - while requirement > 0 OR barrel not empty OR target not full
        // decrement requirement by flow counter
        // truncate flow counter from barrel
        // append flow counter to target
        // break if stopped but not manual
    // if requirement reached 0
        // stop pump
        // close barrel drain tap
        // close dIN tap
        // close target store tap
        // init save 
}

// Storing task
void StoringTask(){
    while (true){ // endless loop Storing task
        // stopped status off?
        // measure barrel 0
            // while barrel 0 not empty
                // measure barrel storing_barrel
                // if not full
                    // Storing(0, storing_barrel, mybarrels.getBarrel(0).measure_ultrasonic() ) // store all barrel 0 amount
                // goto next barrel set storing_barrel += 1
                // all barrels full?
                    // loop - let draining task to kick in when required
            // if barrel 0 empty
                // if drain_counter 
                    // draining status on
                    // keep storing status on so when dtaining finished it goes back here and do "else"
                // else
                    // filling status on
                    // storing status off
    } // endless loop Storing task
}

void Drain(uint16_t barrel, uint16_t requirement){
        // assign flow counter to barrel x
    // open barrel x drain tap + dOUT tap
    // start pump
    // loop while drain counter > 0 and barrel x not empty
    while ( requirement && !barrels.isEmpty(barrel) ){ 
        // truncate flow counter from a barrel
        // decrement requirement by flow counter

        // check for STOPPED state change
        if (SystemState.state_check(STOPPED_STATE) && !SystemState.state_check(MANUAL_STATE)) // break if stopped but not manual
            break;  // breaks the measure loop, stops pump, not decrement the x
                    //stays in the while STOPPED above
        Alarm.delay(1000); 
    }
    // stop pump
    // close barrel x tap
    Alarm.delay(1000); // untill pressure released
    // close dOUT tap
    // init save
    SaveStructs(); 
}

// Draining task
void DrainingTask(){
    while (true){ // endless loop Draining task

        while (Transfers.drain_counter  && SystemState.state_check(DRAINIG_STATE)){ // have a need to drain
            while (SystemState.state_check(STOPPED_STATE)) // stay here if system stopped
                Alarm.delay(1000); // check every second


            // measure barrel x - Barrel x not empty?
            if ( !barrels.isEmpty(Transfers.draining_barrel) ){ 
                Drain(Transfers.draining_barrel, Transfers.drain_requirement);
            }


            else { // still in the draincounter > 0 loop -  barrel x is empty              
                if (Transfers.draining_barrel > 1) // if not the first barrel (i started from last to first)
                    Transfers.draining_barrel--; // goto next barrel
                else { // all storage barrels empty? 
                // unset DRAINING state temporarely so fms can kick in.
                SystemState.state_unset(DRAINIG_STATE);
                SystemState.state_load(); // get the last state back f? m? s? stopped or not..
                // init save 
                SaveStructs();
                // set x back to last barrel
                Transfers.draining_barrel = NUM_OF_BARRELS-1; //  -1 cause we start from zero
                }

            }        
        } //Transfers.drain_counter  && SystemState.state_check(DRAINIG_STATE)
        
        // i"m here because drainCounter reached 0 - or not DRAINIG_STATE
        // if still DRAINIG_STATE then unset DRAINIG_STATE and undo to last state before draining - back to fms
        if (SystemState.state_check(DRAINIG_STATE)) { 
            // set draining status off
            SystemState.state_unset(DRAINIG_STATE);
            // undo to last state before draining
            SystemState.state_load();
            // init save 
            SaveStructs();
        }
        // wait untill draining state set - goes here at system start cause DRAINIG_STATE not set
        if (!SystemState.state_check(DRAINIG_STATE))
            OUT_PORT.println(F("draining task entering wait-mode"));
        while ( !SystemState.state_check(DRAINIG_STATE))
            Alarm.delay(1000);



    } // endless loop Draining task
}


void fmsexTask(){ // Filling Mixing Storing Emptying
// fill
//Filling();
// send sms?
// wait for start - set STOPPED. 
// mix
//Mixing();
// store
//Storing();
// repeat
//Draining();

// if drain command - all others stops
        //remember last state (including stopped) 
        //remove STOPPED state - drain 
        
        //!continue fms tasks

        // what should happen if trying to drain more than available?
        // right now drain is looping endlessly
        // can I pause drain and continue after one fms cycle?

}























// WebServer

//https://github.com/me-no-dev/ESPAsyncWebServer

void setupServer(){


    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.print("Requested: ");
        Serial.println( request->url().c_str() );
        if(!SD.exists("/index.html")){
            SD.end();
            Serial.println("trying to restart SD");
            if(!SD.begin(22)){
            Serial.println("unable to read form SD card");
            request->send(200, "text/html", "<html><body><center><h1>check SD Card please</h1></center></body></html>");
            }
            }
        request->send(SD, "/index2.html", String(), false);
    });

    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
        //request->send(200);
        //request->send(200, "text/plain", "OK");
        }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
            if(!index){
                Serial.printf("\r\nUpload Started: %s\r\n", filename.c_str());
                // open the file on first call and store the file handle in the request object
                request->_tempFile = disk->open("/"+filename, "w");
            }
            if(len) {
                Serial.printf("received chunk [from %u to %u]\r\n", index, len);
                // stream the incoming chunk to the opened file
                request->_tempFile.write(data,len);
            }       
            if(final){
                Serial.printf("\r\nUpload Ended: %s, %u Bytes\r\n", filename.c_str(), index+len);
                request->_tempFile.close();
                request->redirect("/list");
                //request->send(200, "text/plain", "File Uploaded !");
            }
        }
    );


    server.on("/list", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.print("Requested: ");
        Serial.println( request->url().c_str() );
        AsyncResponseStream *response = request->beginResponseStream("text/html");
        File dir = disk->open("/");
        File file = dir.openNextFile();
        response->print("<html><body><ul>");
        while(file){
            response->print("<li>");
            response->printf("<button onclick=\"location=\'/del?f=%s\'\">Delete</button><span>\t</span>", file.name());
            response->printf("<a href=\"down?f=%s\"><b>%s</b></a> \t%u bytes </li>", file.name(), file.name(), file.size());
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
        if(isSD) {
            response->print("<div>filesystem is: SD Card</div>");
            response->print("<button onclick=\"location=\'/switchFS\'\">Switch to SPIFFS</button><br><br>");
        }
        else {
            response->print("<div>filesystem is: SPIFFS</div>");           
            response->print("<button onclick=\"location=\'/switchFS\'\">Switch to SD Card</button><br><br>");
        }
        response->print("</body></html>");
        request->send(response);
    });

    server.on("/switchFS", HTTP_GET, [](AsyncWebServerRequest *request){
        if (isSD){
            Serial.println(F("switching to SPIFFS"));
            disk=&SPIFFS;
            isSD=false;
        }
        else { // IMPORTANT !! add check for filesystem availability
            Serial.println(F("switching to SD Card"));
            disk=&SD;
            isSD=true;
        }
        request->redirect("/list");
    });

    server.on("/del", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.print("Requested: ");
        Serial.println( request->url().c_str() );
        if (request->args() > 0 ) { // Arguments were received
            if (request->hasArg("f")) {
                const char *file = request->arg("f").c_str();
                Serial.printf("Deleting file %s ", file);
                disk->remove(file)?Serial.println("Successfully"):Serial.println("Failed");
            }
        }
        else {
            Serial.println("*server: del received no args");
        }
        request->redirect("/list");
    });

    server.on("/down", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.print("Requested: ");
        Serial.println( request->url().c_str() );
        if (request->args() > 0 ) { // Arguments were received
            if (request->hasArg("f")) {
                const char *file = request->arg("f").c_str();
                Serial.printf("Downloading file %s \r\n", file);
                request->send(*disk, file, "text/plain");
            }
        }
        else {
            request->send(200, "text/plain", "file not found");
        }
    });


    server.on("/manual", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.print("Requested: ");
        Serial.println( request->url().c_str() );
        AsyncResponseStream *response = request->beginResponseStream("text/html");
        response->print("<html><body><ul>");
        response->print("<li>Relays</li>");
        for (uint8_t i=0;i<8;i++){
            response->print("<li>");
            response->printf("<button onclick=\"location=\'/man?f=%u&o=1\'\">fill %u on</button><span> </span>", i, i);
            response->printf("<button onclick=\"location=\'/man?f=%u&o=0\'\">fill %u off</button><span> </span>", i, i);
            response->printf("<button onclick=\"location=\'/man?s=%u&o=1\'\">stor %u on</button><span> </span>", i, i);
            response->printf("<button onclick=\"location=\'/man?s=%u&o=0\'\">stor %u off</button><span> </span>", i, i);
            response->printf("<button onclick=\"location=\'/man?d=%u&o=1\'\">dran %u on</button><span> </span>", i, i);
            response->printf("<button onclick=\"location=\'/man?d=%u&o=0\'\">dran %u off</button><span> </span>", i, i);
            response->print("</li>");
        }
        response->print("<li>RGB LED</li>");
        response->print("<li>");
        for (uint8_t i=0;i<8;i++){
            response->printf("<button onclick=\"location=\'/man?rgb=%u\'\">RGB %u</button><span> </span>", i, i);
        }
        response->print("<li>Flow Sensors</li>");
        response->printf("<li>FS1: %llu pulses %llu Liters %u mL/s %u pulse/L</li>", flow.CounterGet(0), flow.CounterGet(0)/flow.MultGet(0), flow.FlowGet(0), flow.MultGet(0));
        response->printf("<li>FS2: %llu pulses %llu Liters %u mL/s %u pulse/L</li>", flow.CounterGet(1), flow.CounterGet(1)/flow.MultGet(1), flow.FlowGet(1), flow.MultGet(1));
        response->print("<li>Pressure Sensors</li>");
        response->printf("<li>PS1: %u Psi %u multiplier %u offset</li>", pressure.measure(0), pressure.MultiplierGet(0), pressure.OffsetGet(0));
        response->printf("<li>PS2: %u Psi %u multiplier %u offset</li>", pressure.measure(1), pressure.MultiplierGet(1), pressure.OffsetGet(1));
        response->print("</ul>");
        response->print("<button onclick=\"location=location\">reload</button><span> </span>");
        response->print("<button onclick=\"location=\'/reset\'\">reset</button><br><br>");
        response->print("<button onclick=\"location=\'/list\'\">open list filesystem</button>");
        response->print("</body></html>");
        request->send(response);
    });



    server.on("/man", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.print("Requested: ");
        Serial.println( request->url().c_str() );
        if (request->args() > 0 ) { // Arguments were received
            if (request->hasArg("f")) {
                int f = request->arg("f").toInt();
                int o = request->arg("o").toInt();
                Serial.printf("setting FillingRelay %i to %i\r\n", f, o);
                expanders.FillingRelay(f,o);
            }
            if (request->hasArg("s")) {
                int s = request->arg("s").toInt();
                int o = request->arg("o").toInt();
                Serial.printf("setting StoringRelay %i to %i\r\n", s, o);
                expanders.StoringRelay(s,o);
            }
            if (request->hasArg("d")) {
                int d = request->arg("d").toInt();
                int o = request->arg("o").toInt();
                Serial.printf("setting DrainingRelay %i to %i\r\n", d, o);
                expanders.DrainingRelay(d,o);
            }
            if (request->hasArg("rgb")) {
                int rgb = request->arg("rgb").toInt();
                Serial.printf("setting setRGBLED to %i\r\n", rgb);
                expanders.setRGBLED(rgb);
            }
        }
        else {
            request->send(200, "text/plain", "no args!");
        }
        request->redirect("/manual");
    });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
        request->redirect("/manual");
        ESP.restart();
    });


    server.on("/backup", HTTP_GET, [](AsyncWebServerRequest *request){
        char buf [4];
        sprintf (buf, "%03u", Backup());
        //request->send(200, "text/html", buf);
        request->redirect("/list");
    });


    server.on("/restore", HTTP_GET, [](AsyncWebServerRequest *request){
        char buf [4];
        sprintf (buf, "%03u", Restore());
        //request->send(200, "text/html", buf);
        request->redirect("/list");
    });


    server.onNotFound([](AsyncWebServerRequest *request) {
        Serial.print("Requested: ");
        Serial.println( request->url().c_str() );
        AsyncResponseStream *response = request->beginResponseStream("text/html");
        response->addHeader("Server","ESP Async Web Server");
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
        response->printf("<li>Multipart: %s</li>", request->multipart()?"true":"false");
        response->print("</ul>");

        response->print("<h3>Headers</h3>");
        response->print("<ul>");
        int headers = request->headers();
        for(int i=0;i<headers;i++){
        AsyncWebHeader* h = request->getHeader(i);
        response->printf("<li>%s: %s</li>", h->name().c_str(), h->value().c_str());
        }
        response->print("</ul>");

        response->print("<h3>Parameters</h3>");
        response->print("<ul>");
        int params = request->params();
        for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isFile()){
            response->printf("<li>FILE[%s]: %s, size: %u</li>", p->name().c_str(), p->value().c_str(), p->size());
        } else if(p->isPost()){
            response->printf("<li>POST[%s]: %s</li>", p->name().c_str(), p->value().c_str());
        } else {
            response->printf("<li>GET[%s]: %s</li>", p->name().c_str(), p->value().c_str());
        }
        }
        response->print("</ul>");

        response->print("</body></html>");
        //send the response last
        request->send(response);
    });

  // Start server
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();
  Serial.println(F("-Server init"));
} // void setupServer() end




















/*-------- NTP code ----------*/
// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address, byte (&packetBuffer)[NTP_PACKET_SIZE]){
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  UDP.beginPacket(address, NTP_PORT);
  UDP.write(packetBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

time_t getNtpTime(){
  isTimeSync = true; // prevent retrigger untill done
  byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
  IPAddress ntpServerIP; // NTP server's ip address
  while (UDP.parsePacket() > 0) ; // discard any previously received packets
  WiFi.hostByName(NTP_HOSTNAME, ntpServerIP);  // get a random server from the pool
  #ifdef DEBUG
  OUT_PORT.print("NTP time request via ");
  OUT_PORT.print(ntpServerIP);
  #endif
  sendNTPpacket(ntpServerIP, packetBuffer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < NTP_TIMEOUT) {
    uint8_t size = UDP.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      OUT_PORT.println(" Received NTP Response");
      UDP.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      uint16_t DTS = 0;//mySettings.DTS?3600UL:0;
      uint16_t timeZone = 3; // temporary
      return secsSince1900 - 2208988800UL + DTS + timeZone * 3600UL; // added DTS
    }
  }
  OUT_PORT.println("No NTP Response :-(");
  isTimeSync = false; // if failed - set back to false
  return 0; // return 0 if unable to get the time
}



//RTC module
/*
RtcDS3231<TwoWire> Rtc(Wire);



void setupRTC (){
    Rtc.Begin();
    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    if (!Rtc.IsDateTimeValid()) {
        if (Rtc.LastError() != 0) // communication error
              Serial.println(Rtc.LastError());
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
    Serial.println(getDateTimeNow());
	RtcTemperature temp = Rtc.GetTemperature();
    Serial.print(temp.AsFloatDegC());
    Serial.println(" *C");
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




void LoadStructs(){
    if (isSD) Restore(); // in case something is missing in the SD
    Serial.printf("Loading system from %s\r\n", isSD?"SD":"SPIFFS");
    if (disk->exists("/SysState.bin"))
        SystemState.LoadSD();
    else
        OUT_PORT.println("/SysState.bin\tnot exist");
        
    if (disk->exists("/Pressure.bin"))
        pressure.LoadSD();
    else
        OUT_PORT.println("/Pressure.bin\tnot exist");

    if (disk->exists("/Flow.bin"))
        flow.LoadSD();
    else
        OUT_PORT.println("/Flow.bin\tnot exist");

    if (disk->exists("/Barrels.bin"))
        barrels.LoadSD();
    else
        OUT_PORT.println("/Barrels.bin\tnot exist");
}

// reimplement later to save on demand only what 
// this one will be used on system stop? save all...?
void SaveStructs(){
    if (!isSaving){ // prevent concurrent saving
    isSaving = true;
    while(flow.FlowGet(0)); // wait untill no flow
    while(flow.FlowGet(1));
    SystemState.SaveSD();
    
    // reimplement to save ondemand?
    barrels.SaveSD();
    pressure.SaveSD();
    flow.SaveSD();
    isSaving = false;
    }
}








/* initial sequnence */
void setup() {

    // initiate UART0 - serial output
    OUT_PORT.begin(115200);

    // initialize UART2 - serial MUX
    Serial2.begin(9600, SERIAL_8N1);

    //WiFiManager Local intialization. Once its business is done, there is no need to keep it around
    AsyncWiFiManager wifiManager(&server,&dns);
    //wifiManager.resetSettings();
    wifiManager.setConfigPortalTimeout(180); // 3 minutes
    wifiManager.autoConnect("AutoConnectAP"); // will stop here if no wifi connected
    OUT_PORT.printf("ESSID: %s\r\n", WiFi.SSID().c_str());

    //initiate expander ports - I2C wire
    expanders.Init();

    // initiate SD Card - SPI bus + SPIFFS
    initStorage();


    // load structs from SD card
    LoadStructs();

    pressure.setSensor(0, PRESSUR_1_PIN, PS1TOOHIGH_ERROR, PS1TOOLOW_ERROR);
    pressure.setSensor(1, PRESSUR_2_PIN, PS2TOOHIGH_ERROR, PS2TOOLOW_ERROR);

    flow.begin();
    flow.CounterReset(0);
    flow.CounterReset(1);

    //bug? testing..
    expanders.UnlockMUX();

    // initialize uart2 SIM800L modem at MUX port 7?
    modemInit();

    // setup NTP


    //start Web Server
    setupServer();
    ArduinoOTA.setHostname("barrels");
    ArduinoOTA.begin();

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
            Serial.println(String("Task ")+String(taskno)+String(" ")+String(sleeptime));
            delay(500+sleeptime*100);
        }
    }

    void loop() {
    // nope, do nothing here
    vTaskDelay(portMAX_DELAY); // wait as much as posible ...
    }

    */


    SendSMS("System Started");
}

 





  
void loop() {
    // disable loop watchdog - working with tasks only?
    ArduinoOTA.handle();
}