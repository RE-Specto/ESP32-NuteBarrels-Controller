//#include "main.h"
#include "FlowSensor.h"

#include "Expanders.h"
// #include "SystemState.h"
// #include "Barrels.h"
// #include "FlowSensor.h"
// #include "PresureSensor.h"
// #include "WebServer.h"
#include "Filesystem.h"
// #include "Modem.h"
// #include "NTPClient.h"
// #include "FMSD.h"
#include "globals.h"


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

//flow sensor 1 interrupt routine
void IRAM_ATTR FlowSensor1Interrupt()
{
    portENTER_CRITICAL_ISR(&mux);
    Flow.Count(0);
    portEXIT_CRITICAL_ISR(&mux);
}

//flow sensor 2 interrupt routine
void IRAM_ATTR FlowSensor2Interrupt()
{
    portENTER_CRITICAL_ISR(&mux);
    Flow.Count(1);
    portEXIT_CRITICAL_ISR(&mux);
}

// attach interrupts
void FSClass::begin()
{
    #ifdef DEBUG_MORE
    LOG.printf("-Flow: init sensors at pins %u, %u\r\n", FLOW_1_PIN, FLOW_2_PIN);
    #endif
    pinMode(FLOW_1_PIN, INPUT_PULLUP);
    pinMode(FLOW_2_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_1_PIN), FlowSensor1Interrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(FLOW_2_PIN), FlowSensor2Interrupt, FALLING);
    Reset(FRESHWATER);
    Reset(NUTRIENTS);
}

bool FSClass::LoadSD() { return Filesys.Load("/Flow.bin", (byte *)&iFsens, sizeof(iFsens)); }

bool FSClass::SaveSD() { return Filesys.Save("/Flow.bin", (byte *)&iFsens, sizeof(iFsens)); }


// returns flow in mililiters per second for sensor No(sens)
uint16_t FSClass::Get(byte sens)
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    // measurement older than 1 second means no flow
    if ((millis() - iFsens[sens]._lastMilis) > 1000)
        return 0;
    return ((1000000 / (iFsens[sens]._flow ? iFsens[sens]._flow : 1)) / iFsens[sens]._conversion_divider); // in mililiters per second
}                                                                                                          // Guru Meditation Error: Core  1 panic'ed (IntegerDivideByZero). Exception was unhandled.

// executed by interrupt to increase counter
void IRAM_ATTR FSClass::Count(byte sens)
{
    iFsens[sens]._counter++;
    iFsens[sens]._flow = millis() - iFsens[sens]._lastMilis; // interval in miliseconds from the last call
    iFsens[sens]._lastMilis = millis();
}

// get raw flow counter pulses
uint32_t FSClass::Counted(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return iFsens[sens]._counter; 
}

// Subtracts count from sensor (after we applied the count to target barrel)
void FSClass::CounterSubtract(byte sens, uint32_t value)
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    if (iFsens[sens]._counter >= value)
        iFsens[sens]._counter -= value;
    else
    {
        iFsens[sens]._counter = 0;
        LOG.printf("Err: tried to decrease flowsensor%u below zero, with %u\r\n", sens + 1, value);
    }
}

// reset count for sensor (sens)
void FSClass::Reset(uint_fast8_t sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    if (iFsens[sens]._counter)
    {
        LOG.printf("Reseting flowsensor %u counter, from:%u to:0\r\n", sens+1, iFsens[sens]._counter);
        iFsens[sens]._counter = 0; 
        if (!SaveSD()) { LOG.println("[E] unable to Save Reset"); } // just warn for now..
    }
}

// returns pulses to liter
uint16_t FSClass::Divider(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return iFsens[sens]._conversion_divider; 
}

// set pulses to liter (sensor number, divider)
void FSClass::DividerSet(byte sens, uint16_t div) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    if (div)
    {
        LOG.printf("Changing flowsensor %u divider, from:%u to:%u\r\n", sens+1, iFsens[sens]._conversion_divider, div);
        iFsens[sens]._conversion_divider = div; 
        if (!SaveSD()) { LOG.println("[E] unable to Save DividerSet"); } // just warn for now..
    }
    else
    {
        LOG.println("Did anyone just Divided By Zero? Really?");
    }
}

// returns sensor checks state
bool FSClass::Enabled(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return iFsens[sens]._enabled;
}

// sets sensor checks on
void FSClass::Enable(byte sens) 
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    iFsens[sens]._enabled = true; 
    LOG.printf("flowsensor %u Enabled\r\n", sens+1);
    if (!SaveSD()) { LOG.println("[E] unable to Save Enable"); } // just warn for now..
    }

// sets sensor checks off
void FSClass::Disable(byte sens)
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    iFsens[sens]._enabled = false; 
    LOG.printf("flowsensor %u Disabled\r\n", sens+1);
    if (!SaveSD()) { LOG.println("[E] unable to Save Disable"); } // just warn for now..
}

// request Calib() run for sensor
void FSClass::RequestCalib(byte sens)
{
    sens--;
    iFsens[sens]._need_calib = true;
}

// check for Calib() run requests
void FSClass::CheckNeedCalib()
{
    // sizeof iFsens / sizeof iFsens[0]
    // hardcoded for better performance
    for (byte sens = 1; sens <= 2; sens++)  // two sensors
    {
        if (iFsens[sens-1]._need_calib)
        {
            Calib(sens, 5);
        }
    }
}

// fill 5L freshwater for calibration
void FSClass::Calib(byte sens, byte liters)
{
    iFsens[sens-1]._need_calib = false; //prevent double runs
    LOG.printf("Calibrating flow sensor %u, with %u liters.\r\n", sens, liters);
    uint16_t timeLeft = 15000 / portTICK_PERIOD_MS; //hardcoced to 15sec at this point
    Flow.Reset(sens); 
    Expanders.FillingRelay(5, true); // f5 is bypass tap in new system
    Expanders.FillingRelay(0, true);
    uint16_t pulsesIn5L = Flow.Divider(sens) * 5; // 2d0: substract compensation for flow1 enertia
    while (timeLeft && Flow.Counted(sens) < pulsesIn5L)
    {
        // Need timeout protection
        vTaskDelay(1);
        timeLeft--;
    }
    Expanders.FillingRelay(5, false); // f5 is bypass tap in new system
    Expanders.FillingRelay(0, false);
    if (!timeLeft)
    {
        LOG.println("[E] timed out waiting for Calib flow");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    LOG.printf("calib5L pulsed %u expected %u.\r\n", Flow.Counted(1), pulsesIn5L);
    Flow.Reset(sens);
}

FSClass Flow;
