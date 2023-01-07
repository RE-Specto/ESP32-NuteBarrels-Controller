//#include "main.h"
#include "FMSD.h"
#include "globals.h"
#include "Expanders.h"
#include "SystemState.h"
#include "Barrels.h"
#include "FlowSensor.h"
#include "PresureSensor.h"
// #include "WebServer.h"
#include "Filesystem.h"
#include "Modem.h"
// #include "NTPClient.h"
// #include "FMSD.h"

void MixerExternal(bool state)
{
    // Expanders.FillingRelay(POOLS, state);
}

void ServiceManual()
{
    if (State.ManualTask())
    {
        switch (State.ManualTask())
        {
            case 1:
            FillManual();
            break;
            case 2:
            MixManual();
            break;
            case 3:
            StoreManual();
            break;
            case 4:
            DrainManual();
            break;
            case 5:
            BypassManual();
            break;
        }
        LOG.printf("watermark:%u\r\n", uxTaskGetStackHighWaterMark(loop1));
    }
    vTaskDelay(500);
}

// loops until stopped state is unset
void StoppedWait()
{
    while (State.Check(STOPPED_STATE))
        ServiceManual();
}

// closes barrels taps 
// wait until no "stopped state"
// opens taps back
void fmsPause(byte Source, byte Destination)
{
    LOG.println(F("Status Stopped! auto paused\r\n"));
    // if no source barrel = we are filling
    if (Source == 0xFF)
    {
        Expanders.FillingRelay(FRESHWATER_RELAY, false);
        vTaskDelay(500);
        Expanders.FillingRelay(Destination, false);
        StoppedWait();
        Expanders.FillingRelay(FRESHWATER_RELAY, true);
        Expanders.FillingRelay(Destination, true);
    }
    else
    {
        // stop pump
        Expanders.Pump(false);
        // wait until pressure released
        vTaskDelay(500);
        Expanders.FillingRelay(Destination, false);
        StoppedWait();
        // open pool x fill tap
        Expanders.FillingRelay(Destination, true);
        // start pump
        Expanders.Pump(true);
    }
}

// receives sensor number
// sets stopped state if pressure error
void PressureCheck(byte sens)
{
    Pressure.measure(sens);
    switch (Pressure.Errors(sens))
    {
        case PRESSURE_NORMAL:
        break;
        case PRESSURE_NOPRESSURE:
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        Pressure.measure(sens);
        if (Pressure.Errors(sens) == PRESSURE_NOPRESSURE)
        {
            LOG.printf("no pressure @sensor %u\r\n", sens);
            State.Set(STOPPED_STATE);
        }
        break;
        case PRESSURE_OVERPRESSURE:            
        case PRESSURE_DISCONNECT:            
        case PRESSURE_SHORTCIRCUIT:            
        State.Set(STOPPED_STATE);
        break;
    }
}

// receives sensor number
// sets stopped state if no flow
void FlowCheck(byte sens)
{
    if (!Flow.Get(FRESHWATER))
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        if (!Flow.Get(FRESHWATER))
        {
            State.Set(STOPPED_STATE);
            Expanders.setRGBLED(LED_GREEN);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            Expanders.setRGBLED(LED_YELLOW);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            Expanders.setRGBLED(LED_RED);
            LOG.println("No flow");
            if (sens == FRESHWATER)
            {
                if (!State.isError(ERR_WATER_NOFLOW))
                {
                    State.SetError(ERR_WATER_NOFLOW);
                    SendSMS("error: no water flow while Filling. check flowsensor 1, filling solenoid");                
                }
            }
            else
            {
                if (!State.isError(ERR_NUTRI_NOFLOW))
                {
                    State.SetError(ERR_NUTRI_NOFLOW);
                    SendSMS("error: no nutrients flow. check pump, flowsensor 2, solenoids");                
                }
            }
        }
    }
}

void blinkDelay(unsigned long ms, byte color)
{
    vTaskDelay(ms / 2 / portTICK_PERIOD_MS);
    if (State.ManualTask()) // blinks differently for auto and manual
        Expanders.setRGBLED(LED_WHITE);
    else
        Expanders.setRGBLED(LED_OFF);
    vTaskDelay(ms / 2 / portTICK_PERIOD_MS);
    Expanders.setRGBLED(color);
}


// checks whatever clean water line have pressure
// fills clean water until FillRequirement, or barrel is full,
// stops if flowsensor malfunction detected
void Fill()
{
    Serial.println();
    LOG.printf("filling barrel:%u to %uL\r\n", 0, State.FillRequirement());
    Flow.Reset(FRESHWATER); // reset flow counter 1
    Expanders.FillingRelay(0, true); // open barrel filling tap
    Expanders.FillingRelay(FRESHWATER_RELAY, true);
    // fill until requirement OR barrel_high_level
    // while (!Barrels.isFillTargetReached(barrel, FRESHWATER, requirement))
    while (Barrels.FreshGet(0) < State.FillRequirement())
    { 
        // LOGIC
        Barrels.FreshwaterFillCalc(0); // apply flowcount to barrel         
        if (Barrels.isFull(0) && !Barrels.Errors(0)) // if barrel ammount is Full then stop
        {
            LOG.printf("[E] barrel:%u full. breaking..\r\n", 0);
            break;
        }
        if (Barrels.Errors(0))
        {
            LOG.printf("Barrel %u error state %u auto paused.\r\n", 0, Barrels.Errors(0));
            State.Set(STOPPED_STATE);
        }
        // STOP-CHECK
        if (State.Check(STOPPED_STATE))
            fmsPause(0xff, 0);
        // SENSOR CHECK
        if (Pressure.Enabled(FRESHWATER))
            PressureCheck(FRESHWATER);
        if (Flow.Enabled(FRESHWATER))
            FlowCheck(FRESHWATER);
        blinkDelay(1000, LED_YELLOW);
    }
    Expanders.FillingRelay(FRESHWATER_RELAY, false); // close tap
    vTaskDelay(500);
    Expanders.FillingRelay(0, false);
    Barrels.FreshwaterFillCalc(0);  // apply flowcount to barrel
    Flow.Reset(FRESHWATER); // reset flow counter 1
    LOG.printf("END filling barrel:%u to %uL\r\n", 0, State.FillRequirement());
    Serial.println();
}

// will mix until "duration" minutes is over,
// or before if state changed to stopped while we're not operating manual
void Mix()
{
    Serial.println();
    LOG.printf("mixing for %uMin.\r\n", State.MixTimer());
    // #ifdef MIX_INTERNAL
    Flow.Reset(NUTRIENTS); // reset flow counter 2
    // open mixer fill tap
    Expanders.FillingRelay(0, true);
    Expanders.Pump(true);
    // #endif
    // #ifdef MIX_EXTERNAL
    // MixerExternal(true); // turn on optional mixer motor at drain6
    // #endif
    // loop - while mix timer > 0
    while (State.MixTimer())
    {
        // LOGIC
        LOG.printf("mixing: %uMin remaining\r\n", State.MixTimer());
        for (byte s=0;s<60;s++)
        {
            // STOP-CHECK
            if (State.Check(STOPPED_STATE))
                // MixerExternal(false); // turn off optional mixer motor at drain6
                // #ifdef MIX_INTERNAL
                fmsPause(0, 0);
                // #else
                // LOG.println(F("Status Stopped! auto paused\r\n"));
                // StoppedWait();
                // #endif
                // #ifdef MIX_EXTERNAL
                // MixerExternal(true); // turn on optional mixer motor at drain6
                // #endif
            // SENSOR CHECK
            if (Pressure.Enabled(NUTRIENTS))
                PressureCheck(NUTRIENTS);
            #ifdef MIX_INTERNAL
            if (Flow.Enabled(NUTRIENTS))
                FlowCheck(NUTRIENTS);
            #endif
            blinkDelay(1000, LED_GREEN);
        }
        State.MixLess();// decrement counter every minute
    }
    // counter reached zero

    // make freshwater into nutrients
    Barrels.WaterToNutrients(0);

    // MixerExternal(false); // turn off optional mixer motor at drain6
    // #ifdef MIX_INTERNAL
    Expanders.Pump(false);
    Expanders.FillingRelay(0, false);
    // report mixed ammount
    LOG.printf("mixed %u liters\r\n", Flow.Counted(NUTRIENTS) / Flow.Divider(NUTRIENTS));
    Flow.Reset(NUTRIENTS); // reset flow counter 2
    // #endif
    State.MixReset(); // reset mix timer for next run
    LOG.println("END mixing.");
    // Serial.println();
}

// transfers nutrients from "barrel" to "target"
// until barrel is empty or until target is full
// or until "stopped" state set, except if system state also set to manual
void Store(byte barrel, byte target)
{
    // Serial.println();
    // LOG.printf("storing from barrel:%u to target %u\r\n", barrel, target);
    // Flow.Reset(NUTRIENTS); // reset flow counter 2
    // if (barrel == target)
    // {
    //     LOG.println("Error! trying to store to itself.\r\nstoring function exit now.");
    //     return; // exit right away
    // }
    // OpenTaps(barrel, target); // drain barrel into target
    // // loop - while source barrel not empty AND target not full
    // while (!Barrels.isEmpty(barrel) && !Barrels.isFull(target))
    // {
    //     // LOGIC
    //     // every 50 liters (not too often for good calculation accuracy)
    //     if (Flow.Counted(NUTRIENTS) / Flow.Divider(NUTRIENTS) > 50)
    //         Barrels.NutrientsTransferCalc(barrel, target);
    //     if (Barrels.Errors(barrel))
    //     {
    //         LOG.printf("Barrel %u error state %u auto paused.\r\n", barrel, Barrels.Errors(barrel));
    //         State.Set(STOPPED_STATE);
    //     }
    //     if (Barrels.Errors(target))
    //     {
    //         LOG.printf("Target %u error state %u auto paused.\r\n", target, Barrels.Errors(target));
    //         State.Set(STOPPED_STATE);
    //     }
    //     // STOP-CHECK
    //     if (State.Check(STOPPED_STATE))
    //         fmsPause(barrel, target);
    //     // SENSOR CHECK
    //     if (Pressure.Enabled(NUTRIENTS))
    //         PressureCheck(NUTRIENTS);
    //     if (Flow.Enabled(NUTRIENTS))
    //         FlowCheck(NUTRIENTS);
    //     blinkDelay(1000, LED_CYAN);
    // }
    // CloseTaps(barrel, target);
    // Barrels.NutrientsTransferCalc(barrel, target); // calculate final ammount
    // Flow.Reset(NUTRIENTS); // reset flow counter 2
    // LOG.printf("END storing from barrel:%u to target %u\r\n", barrel, target);
    // Serial.println();
}

// draining from mixer, until barrel is empty
// or until state set to stopped except if state is also manual
void Drain()
{
    byte target = State.StoreBarrel();
    // Serial.println();
    uint16_t barrel_before = Barrels.NutriGet(0);
    LOG.printf("Draining to pool:%u (%uL)\r\n", target, barrel_before);
    Flow.Reset(NUTRIENTS); // reset flow counter 2
    // open pool x fill tap
    Expanders.FillingRelay(target, true);
    Expanders.Pump(true);
    // loop while drain counter > 0 and barrel x not empty
    while (!Barrels.isEmpty(0))
    {
        // LOGIC
        Barrels.NutrientsTransferCalc(0, target);
        if (Barrels.Errors(target))
        {
            LOG.printf("Barrel %u error state %u auto paused.\r\n", target, Barrels.Errors(target));
            State.Set(STOPPED_STATE);
        }
        // STOP-CHECK
        if (State.Check(STOPPED_STATE))
            fmsPause(0, target);
        // SENSOR CHECK
        if (Pressure.Enabled(NUTRIENTS))
            PressureCheck(NUTRIENTS);
        if (Flow.Enabled(NUTRIENTS))
            FlowCheck(NUTRIENTS);
        blinkDelay(1000, LED_BLUE);
    }
    Expanders.Pump(false);
    vTaskDelay(1000);
    Expanders.FillingRelay(target, false);
    vTaskDelay(1000);
    Barrels.NutrientsTransferCalc(0, target);
    Flow.Reset(NUTRIENTS); // reset flow counter 2
     LOG.printf("END Draining. Drained %uL to pool:%u\r\n", barrel_before - Barrels.NutriGet(0), target);
    // Serial.println();
}

// bypass freshwater into selected pool (_storing_barrel)
// until _bypass_req is 0
// decrease _bypass_req using FreshwaterFillCalc
void Bypass()
{
    byte target = State.StoreBarrel();
    Serial.println();
    LOG.printf("filling pools with %uL\r\n", State.BypassMore());
    Flow.Reset(FRESHWATER); // reset flow counter 1
    Expanders.FillingRelay(target, true); // open pool tap
    Expanders.FillingRelay(FRESHWATER_RELAY, true); // open filling tap
    // fill, decrement requirement
    while (State.BypassMore()) // implement !Barrels.isFillTargetReached(POOLS, FRESHWATER, requirement) instead? + FreshwaterFillCalc...
    {
        // LOGIC
        Barrels.FreshwaterFillCalc(target); // apply flowcount to pool freshwater count         
        // State.BypassRecalc();      
        // STOP-CHECK
        if (State.Check(STOPPED_STATE))
            fmsPause(0xff, target); // from filling to pool x
        // SENSOR CHECK
        if (Pressure.Enabled(FRESHWATER))
            PressureCheck(FRESHWATER);
        if (Flow.Enabled(FRESHWATER))
            FlowCheck(FRESHWATER);
        blinkDelay(1000, LED_MAGENTA);
    }
    Expanders.FillingRelay(FRESHWATER_RELAY, false); // close filling tap
    vTaskDelay(1000); // wait for pressure releif
    Expanders.FillingRelay(target, false); // close pool tap
    // Expanders.StoringRelay(target, false);
    LOG.printf("END filling pools with %uL\r\n", Flow.Counted(FRESHWATER) / Flow.Divider(FRESHWATER)); // requirement
    Flow.Reset(FRESHWATER); // reset flow counter 1
    // Serial.println();
}

void fmsTask(void * pvParameters)
{ // Filling Mixing Storing Draining
    //LOG.printf(" fmsd begin  system state:%u  watermark:%u\r\n", State.Get(), uxTaskGetStackHighWaterMark(loop1));
    while (true)
    {
        // !! new fmdb here !!


        
        // !! old fmsd here !!
        // if (State.Check(FILLING_STATE)) // Filling
        // {
        //     LOG.print(F("system running auto - waiting for nutes\r\n"));
        //     State.Set(STOPPED_STATE); // wait until nutes loaded before filling+mixing
        //     while (State.Check(STOPPED_STATE)) // stay here while system is waiting
        //         ServiceManual();
        //     Fill(State.FillBarrel(), State.FillRequirement());
        //     State.Set(MIXING_STATE);
        //     State.Unset(FILLING_STATE);
        // }

        // vTaskDelay(1000 / portTICK_PERIOD_MS); // good idea to wait after each state change

        // if (State.Check(MIXING_STATE)) // Mixing
        // {
        //     Mix(State.FillBarrel());
        //     State.Set(STORING_STATE);
        //     State.Unset(MIXING_STATE);
        // }

        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        // if (State.Check(STORING_STATE)) // Store + Drain
        // {
        //     Expanders.setRGBLED(LED_CYAN);
        //     // still have nutes to transfer
        //     while (!Barrels.isEmpty(State.FillBarrel()))
        //     { // allways lands here first after fill mix
        //         // first we drain if neccesery
        //         if (State.DrainMore())
        //         {
        //             State.Set(DRAINIG_STATE);
        //             vTaskDelay(1000 / portTICK_PERIOD_MS);
        //             // drain until empty or requirement satisfied.
        //             Drain(State.FillBarrel(), State.DrainMore());
        //             Filesys.SaveStructs();
        //             Bypass(); // fill pools with freshwater 
        //             Filesys.SaveStructs();
        //             if (Barrels.isEmpty(State.FillBarrel()))
        //                 break; // if drained filling barrel to empty - break store loop
        //         }
        //         else // no drain requirement
        //         {
        //             State.Unset(DRAINIG_STATE);                    
        //         }

        //         // then we store what is left
        //         // target not full - transfer into storing barrel
        //         // excluding the case where all system was full and storing_barrel pointed to barrel 0
        //         // skip barrel if disabled or errorous!
        //         byte stor = State.StoreBarrel();
        //         if (!Barrels.isFull(stor) && !Barrels.Errors(stor) && stor > 0)
        //         {
        //             Store(State.FillBarrel(), stor);
        //             Filesys.SaveStructs(); 
        //         }
        //         // target full - goto next barrel
        //         else if (stor > 1)
        //         {
        //             LOG.printf("barrel %u busy (e-state %u), skipping to %u..\r\n", stor, Barrels.Errors(stor), stor - 1);
        //             State.MoveStoreDown();
        //         }
        //         else
        //         { // no more next - all full
        //             Expanders.setRGBLED(LED_MAGENTA);
        //             LOG.println("No more empty barrels. system stopped. drain to continue");
        //             // wait for drain request
        //             while (!State.DrainMore())
        //                 ServiceManual();
        //             State.Set(DRAINIG_STATE);
        //             // drain the mixer first
        //             State.SetStoreBarrel(State.FillBarrel());
        //             // drain until empty or requirement satisfied.
        //             Drain(State.FillBarrel(), State.DrainMore());
        //             Filesys.SaveStructs(); 
        //             Bypass(); // fill pools with freshwater 
        //             Filesys.SaveStructs();
        //         }
        //         vTaskDelay(1000 / portTICK_PERIOD_MS);
        //     } // got here cause filling_barrel is empty

        //     while (State.DrainMore())
        //     {
        //         State.Set(DRAINIG_STATE);
        //         vTaskDelay(1000 / portTICK_PERIOD_MS);
        //         // storing_barrel not empty? not errorous? drain it
        //         if (!Barrels.isEmpty(State.StoreBarrel()) && !Barrels.Errors(State.StoreBarrel()))
        //         {
        //             Drain(State.StoreBarrel(), State.DrainMore());
        //             Filesys.SaveStructs(); 
        //             Bypass(); // fill pools with freshwater 
        //             Filesys.SaveStructs();
        //         }
        //         // storing_barrel empty but not the last barrel (i filled from last to first)  // try next barrel
        //         else if (State.StoreBarrel() < NUM_OF_BARRELS - 1)
        //             State.MoveStoreUp();
        //         // all storage barrels empty?
        //         else
        //         {
        //             LOG.println("No more non-empty barrels to drain. doing another fill cycle");
        //             break; // totally empty - will do another cycle fms to refill
        //         }
        //     }
        //     if (!State.DrainMore())
        //         State.Unset(DRAINIG_STATE);
        //     // repeat the fms cycle if no draining required, or all barrels empty
        //     State.Set(FILLING_STATE);
        //     State.Unset(STORING_STATE);
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        // } // if storing_state
    }     // endless loop ends here :)
}

void FillManual()
{
    LOG.println("Manual Filling");
    byte barrel = State.ManualDestination();
    Flow.Reset(FRESHWATER); 
    Expanders.FillingRelay(barrel, true);
    Expanders.FillingRelay(FRESHWATER_RELAY, true);
    while (State.ManualAmmount())
    {
        // LOGIC
        Barrels.FreshwaterFillCalc(barrel); // apply flowcount to barrel         
        if (Barrels.isFull(barrel))
        {
            LOG.printf("[E] barrel:%u full. breaking..\r\n", barrel);
            break;
        }
        // MAN Break
        if (!State.ManualTask())
            break;
        blinkDelay(1000, LED_YELLOW);
    }
    Expanders.FillingRelay(FRESHWATER_RELAY, true);
    vTaskDelay(1000); // wait for pressure releif
    Expanders.FillingRelay(barrel, false);
    vTaskDelay(1000); // wait for pressure releif
    Barrels.FreshwaterFillCalc(barrel);
    Flow.Reset(FRESHWATER);
    State.ResetManual(); // important - no double-run
    Expanders.setRGBLED(LED_WHITE);
}

void MixManual()
{
    LOG.println("Manual Mixing");
    byte barrel = State.ManualDestination();
    Flow.Reset(NUTRIENTS);
    Expanders.FillingRelay(0, true);
    Expanders.Pump(true);
    while (State.MixTimer())
    {
        LOG.printf("barrel:%u manual %uMin remaining\r\n", barrel, State.MixTimer());
        // LOGIC
        for (byte s=0;s<60;s++)
        {
            // MAN Break
            if (!State.ManualTask())
                break;
            blinkDelay(1000, LED_GREEN);
        }
        State.MixLess();// decrement counter every minute
    }
    // stop pump
    Expanders.Pump(false);
    // wait until pressure released
    vTaskDelay(1000);
    Expanders.FillingRelay(0, false);
    Flow.Reset(NUTRIENTS);
    State.ResetManual(); // important - no double-run
    Expanders.setRGBLED(LED_WHITE);
}

void StoreManual()
{
    // LOG.println("Manual Storing");
    // byte barrel = State.ManualSource();
    // byte target = State.ManualDestination();
    // Flow.Reset(NUTRIENTS);
    // if (barrel == target)
    // {
    //     LOG.println("Error! trying to store to itself.\r\nstoring function exit now.");
    //     State.ResetManual(); // important - no double-run
    //     return; // exit right away
    // }
    // OpenTaps(barrel, target); // drain barrel into target
    // // loop - while source barrel not empty AND target not full
    // while (!Barrels.isEmpty(barrel) && !Barrels.isFull(target))
    // {
    //     // LOGIC
    //     // every 50 liters (not too often for good calculation accuracy)
    //     if (Flow.Counted(NUTRIENTS) / Flow.Divider(NUTRIENTS) > 50)
    //         Barrels.NutrientsTransferCalc(barrel, target);
    //     // MAN Break
    //     if (!State.ManualTask())
    //         break;
    //     blinkDelay(1000, LED_CYAN);
    // }
    // CloseTaps(barrel, target);
    // Barrels.NutrientsTransferCalc(barrel, target);
    // Flow.Reset(NUTRIENTS); 
    // State.ResetManual(); // important - no double-run
    // Expanders.setRGBLED(LED_WHITE);
}

void DrainManual()
{
    // LOG.println("Manual Draining");
    // byte barrel = State.ManualSource();
    // Flow.Reset(NUTRIENTS);
    // OpenTaps(barrel, POOLS);
    // // loop while drain counter > 0 and barrel x not empty
    // while (State.DrainMore() && !Barrels.isEmpty(barrel))
    // {
    //     // LOGIC
    //     State.DrainRecalc(barrel);
    //     // MAN Break
    //     if (!State.ManualTask())
    //         break;
    //     blinkDelay(1000, LED_BLUE);
    // }
    // CloseTaps(barrel, POOLS);
    // vTaskDelay(1000);
    // State.DrainRecalc(barrel); // last flow calculation
    // Flow.Reset(NUTRIENTS);
    // State.ResetManual(); // important - no double-run
    // Expanders.setRGBLED(LED_WHITE);
    byte target = State.ManualDestination();
    uint16_t barrel_before = Barrels.NutriGet(0);
    LOG.printf("Manual Draining to pool:%u (%uL)\r\n", target, barrel_before);
    Flow.Reset(NUTRIENTS); // reset flow counter 2
    Expanders.FillingRelay(target, true);
    Expanders.Pump(true);
    // loop while manual ammount > 0 and barrel 0 not empty
    while (State.ManualAmmount() && !Barrels.isEmpty(0))
    {
        // LOGIC
        Barrels.NutrientsTransferCalc(0, target);
        // MAN Break
        if (!State.ManualTask())
            break;
        blinkDelay(1000, LED_BLUE);
    }
    Expanders.Pump(false);
    vTaskDelay(1000);
    Expanders.FillingRelay(target, false);
    vTaskDelay(1000);
    Barrels.NutrientsTransferCalc(0, target);
    Flow.Reset(NUTRIENTS); // reset flow counter 2
    LOG.printf("END Manual Draining. Drained %uL to pool:%u\r\n", barrel_before - Barrels.NutriGet(0), target);
}

// bypass freshwater to pools
void BypassManual()
{
    LOG.println("Manual Water to pools");
    byte target = State.ManualDestination();
    Flow.Reset(FRESHWATER); // reset flow counter 1
    Expanders.FillingRelay(target, true);
    Expanders.FillingRelay(FRESHWATER_RELAY, true);
    while (State.ManualAmmount()) // maybe need to stop at +1 if enertia?
    {
        // LOGIC
        Barrels.FreshwaterFillCalc(target); // apply flowcount to barrel         
        // MAN Break
        if (!State.ManualTask())
            break;
        blinkDelay(1000, LED_MAGENTA);
    }

    Expanders.FillingRelay(FRESHWATER_RELAY, false);
    vTaskDelay(1000);
    Expanders.FillingRelay(target, false);
    vTaskDelay(1000);
    Barrels.FreshwaterFillCalc(target); // apply flowcount to barrel         
    Flow.Reset(FRESHWATER);
    State.ResetManual(); // important - no double-run
    Expanders.setRGBLED(LED_WHITE);
}

