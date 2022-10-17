//#include "main.h"
#include "SystemState.h"
#include "Expanders.h"
// #include "SystemState.h"
#include "Barrels.h"
#include "FlowSensor.h"
// #include "PresureSensor.h"
// #include "WebServer.h"
#include "Filesystem.h"
#include "Modem.h"
// #include "NTPClient.h"
// #include "FMSD.h"
#include "globals.h"


void StatClass::begin()
{
    #ifdef DEBUG_MORE
    LOG.printf("-System: init start stop at pins %u, %u\r\n", START_PIN, STOP_PIN);
    #endif
    pinMode(START_PIN, INPUT_PULLUP);
    // pinMode(STOP_PIN, INPUT_PULLUP); // already pulled up by hardware
    attachInterrupt(digitalPinToInterrupt(START_PIN), StartButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(STOP_PIN), StopButtonInterrupt, FALLING);
    // State.SetStoreBarrel(State.FillBarrel()); //storing disabled. //(NUM_OF_BARRELS -1 cause we start from zero)
}

bool StatClass::LoadSD() { return Filesys.Load("/SysState.bin", (byte *)&iState, sizeof(iState)); }

bool StatClass::SaveSD() { return Filesys.Save("/SysState.bin", (byte *)&iState, sizeof(iState)); }

void IRAM_ATTR StartButtonInterrupt()
{
    portENTER_CRITICAL_ISR(&mux);
    State.Unset(STOPPED_STATE);
    State.ResetManual();
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR StopButtonInterrupt()
{
    portENTER_CRITICAL_ISR(&mux);
    State.Set(STOPPED_STATE);
    State.ResetManual();
    portEXIT_CRITICAL_ISR(&mux);
}

// returns system state raw integer value (0=uninitialized)
byte StatClass::Get() { return iState._state_now; }

// set state using mask (FILLING_STALE,STORING_STATE....)
void IRAM_ATTR StatClass::Set(byte mask) { iState._state_now |= mask; }

// unset mask from state  (FILLING_STALE,STORING_STATE....)
void IRAM_ATTR StatClass::Unset(byte mask) { iState._state_now &= ~mask; }

// dangerous - override system state
void StatClass::Override(byte state) 
{ 
    LOG.printf("Overriding system state from:%u to:%u\r\n", iState._state_now, state);
    iState._state_now = state; 
}

// preserve prevoius state
void StatClass::Apply() { iState._state_before = iState._state_now; }

// restore previous state
void StatClass::Revert() { iState._state_now = iState._state_before; }

// returns true if state have "mask-bit" state on. ex: Check(MIXING_STATE);
bool StatClass::Check(byte mask) { return iState._state_now & mask; }

void StatClass::Print()
{
    LOG.printf("system state:%u ", Get());
    // keep it simple
    if (Check(FILLING_STATE))
        Serial.print(F("FILLING "));
    if (Check(MIXING_STATE))
        Serial.print(F("MIXING "));
    if (Check(STORING_STATE))
        Serial.print(F("STORING "));
    if (Check(DRAINIG_STATE))
        Serial.print(F("DRAINIG "));
    if (Check(STOPPED_STATE))
        Serial.print(F("STOPPED "));
    Serial.println();
}

// detects and prints system state change
bool StatClass::isChanged()
{
    if (iState._state_now != iState._state_before)
    {
        Apply();
        Print();
        if (!digitalRead(START_PIN) && !digitalRead(STOP_PIN))
        {
            LOG.println("Start and Stop pressed together");
            State.Set(STOPPED_STATE);
            Apply(); // prevent double trigger
            if (!Expanders.Protect())
                Expanders.Protect(true);
            vTaskDelay(1000);
            State.Set(STOPPED_STATE); // set again in case it changed by interrupt
            Apply();
        }
        if (!digitalRead(START_PIN) && digitalRead(STOP_PIN))
        {
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            if (!digitalRead(START_PIN) && digitalRead(STOP_PIN))
            {
                LOG.println("Start long-pressed");
                if (Expanders.Protect())
                    Expanders.Protect(false);
            }
        }
        if (Expanders.Protect())
        {
            State.Set(STOPPED_STATE);
            Apply(); // prevent double trigger
            LOG.println("Error! system in protect state. long-press start to unprotect");
        }
        Filesys.SaveStructs();
        return true;
    }
    return false;
}

// returns error state
uint16_t StatClass::Errors() { return iState._error_now; }

// returns true if error have "mask-bit" state on. ex: Error(ERR_WATER_NOFLOW);
bool StatClass::isError(uint16_t error) { return iState._error_now & error; }

// set error using mask
void StatClass::SetError(uint16_t error) { iState._error_now |= error; }

// unset error using mask
void StatClass::UnsetError(uint16_t error) { iState._error_now &= ~error; }

// Dangerous - override error state
void StatClass::OverrideError(uint16_t error) 
{
    LOG.printf("Overriding error state from:%u to:%u\r\n", iState._error_now, error);
    iState._error_now = error;
}

uint16_t StatClass::FillRequirement() {return iState._fill_req;}

void StatClass::SetFillReq(uint16_t req) 
{
    LOG.printf("Changing fill requirement from:%u to:%u\r\n", iState._fill_req, req);
    iState._fill_req = req;
}

byte StatClass::FillBarrel() {return iState._filling_barrel;}

void StatClass::SetFillBarrel(byte barrel) 
{
    LOG.printf("Changing fill barrel from:%u to:%u\r\n", iState._filling_barrel, barrel);
    iState._filling_barrel = barrel;
}

uint16_t StatClass::MixRequirement() {return iState._mix_req;}

void StatClass::SetMixReq(uint16_t req) 
{
    LOG.printf("Changing mix requirement from:%u to:%u\r\n", iState._mix_req, req);
    iState._mix_req = req;
}

// returns time left to mix
uint16_t StatClass::MixTimer() 
{
    if (iState._manual_task)
        return iState._manual_ammo;
    else
        return iState._mix_timer;
}

// decreases mix timer
void StatClass::MixLess() 
{
    if (iState._manual_task)
    {
        if (iState._manual_ammo)
            iState._manual_ammo--;
    }
    else
        if (iState._mix_timer) // prevent integer overflow
            iState._mix_timer--;
}

// resets mix timer to default value
void StatClass::MixReset()
{
    LOG.printf("resetting mix timer from:%u to:%umin.\r\n", iState._mix_timer, iState._mix_req);  
    iState._mix_timer = iState._mix_req;
}

byte StatClass::StoreBarrel() {return iState._storing_barrel;}

void StatClass::SetStoreBarrel(byte barrel) 
{
    LOG.printf("Changing store barrel from:%u to:%u\r\n", iState._storing_barrel, barrel);
    iState._storing_barrel = barrel;
}

void StatClass::MoveStoreUp() 
{
    iState._storing_barrel++;
    LOG.printf("Storing barrel inc, now barrel#%u\r\n", iState._storing_barrel);
}

void StatClass::MoveStoreDown()
{
    if (iState._storing_barrel) // prevent integer overflow
    {
        iState._storing_barrel--;
        LOG.printf("Storing barrel dec, now barrel#%u\r\n", iState._storing_barrel);
    }
    else
    {
        LOG.println("[e] trying to decrease store barrel below zero!!");
    }
}

void StatClass::SetDrainReq(uint16_t req) 
{
    LOG.printf("Changing drain requirement from:%u to:%u\r\n", iState._drain_req, req);
    iState._drain_req = req;
}

void StatClass::SetBypassReq(uint16_t req) 
{
    LOG.printf("Changing bypass requirement from:%u to:%u\r\n", iState._bypass_req, req);
    iState._bypass_req = req;
}

// returns how much to drain
uint16_t StatClass::DrainMore()
{
    if (iState._manual_task)
        return iState._manual_ammo;
    else
        return iState._drain_req;
}

// returns how much to bypass
uint16_t StatClass::BypassMore()
{
    if (iState._manual_task)
        return iState._manual_ammo;
    else
        return iState._bypass_req;
}

// substract flow sensor counted liters from drain requirement
// increase total counted drain
// substract counted ammount from flow counter
void StatClass::DrainRecalc(byte barrel)
{
    #ifdef DEBUG_MORE
    LOG.println(__FUNCTION__);
    #endif
    // flow counter may be increased during calculation 
    // because flowsensor works on interrupts
    // so capturing only the current state for count
    uint32_t liters = Flow.Counted(NUTRIENTS) / Flow.Divider(NUTRIENTS);
    if (liters) // quotient only, remainder stays on sensor for next recalc
    {
        if (iState._manual_task)
        {
            if (iState._manual_ammo > liters) // prevent integer overflow
                iState._manual_ammo -= liters;
            else
                iState._manual_ammo = 0;
        }
        else
        {
            if (iState._drain_req > liters) // prevent integer overflow
                iState._drain_req -= liters;
            else
                iState._drain_req = 0;
        }
        if ((iState._drain_total+liters)<UINT32_MAX) // prevent integer overflow
            iState._drain_total += liters;
        else
            iState._drain_total = UINT32_MAX;
        Barrels.NutriLess(barrel, liters);
        // decrement flow counter by "counted so far" pulses (liters times divider)
        Flow.CounterSubtract(NUTRIENTS, liters * Flow.Divider(NUTRIENTS)); 
    }
}

// substract flow sensor counted liters from bypass requirement
// increase total counted bypass
// substract counted ammount from flow counter
void StatClass::BypassRecalc()
{
    #ifdef DEBUG_MORE
    LOG.println(__FUNCTION__);
    #endif
    uint32_t liters = Flow.Counted(FRESHWATER) / Flow.Divider(FRESHWATER);
    if (liters) // quotient only, remainder stays on sensor for next recalc
    {
        if (iState._manual_task)
        {
            if (iState._manual_ammo > liters) // prevent integer overflow
                iState._manual_ammo -= liters;
            else
                iState._manual_ammo = 0;
        }
        else
        {
            if (iState._bypass_req > liters) // prevent integer overflow
                iState._bypass_req -= liters;
            else
                iState._bypass_req = 0;
        }
        if ((iState._bypass_total+liters)<UINT32_MAX) // prevent integer overflow
            iState._bypass_total += liters;
        else
            iState._bypass_total = UINT32_MAX;
        // decrement flow counter by "counted so far" pulses (liters times divider)
        Flow.CounterSubtract(FRESHWATER, liters * Flow.Divider(FRESHWATER));
    }
}

uint32_t StatClass::DrainTotal()
{
    return iState._drain_total;
}

uint32_t StatClass::BypassTotal()
{
    return iState._bypass_total;
}

void StatClass::SetManual(byte task, byte src, byte dest, uint16_t ammo)
{
    iState._manual_task=task;
    iState._manual_src=src;
    iState._manual_dest=dest;
    iState._manual_ammo=ammo;
}

void IRAM_ATTR StatClass::ResetManual(){iState._manual_task=0;}

byte StatClass::ManualTask() {return iState._manual_task;}

byte StatClass::ManualSource() {return iState._manual_src;}

byte StatClass::ManualDestination() {return iState._manual_dest;}

uint16_t StatClass::ManualAmmount() {return iState._manual_ammo;}

StatClass State;
